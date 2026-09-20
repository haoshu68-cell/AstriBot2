#include "voxelslam.hpp"
#include <filesystem>

using namespace std;

class ResultOutput
{
public:
  static ResultOutput &instance()
  {
    static ResultOutput inst;
    return inst;
  }

  // aft_mapped tf is now published at 20Hz by HighRateOdom (see
  // highrate_odom.hpp), anchored to x_curr right after each scan's
  // optimization finishes (see the anchor() calls around the two
  // pub_localtraj() call sites) instead of being broadcast here at raw
  // LiDAR-frame rate.

  void pub_localtraj(PLV(3) &pwld, double jour, IMUST &x_curr, int cur_session, pcl::PointCloud<PointType> &pcl_path)
  {
    pcl::PointCloud<PointType> pcl_send;
    pcl_send.reserve(pwld.size());
    for(Eigen::Vector3d &pw: pwld)
    {
      Eigen::Vector3d pvec = pw;
      PointType ap;
      ap.x = pvec.x();
      ap.y = pvec.y();
      ap.z = pvec.z();
      pcl_send.push_back(ap);
    }
    pub_pl_func(pcl_send, pub_scan, x_curr.t);

    // Height-ROI-filtered copy of the same scan, for navigation obstacle
    // detection (/map_scan_filtered). Assumes flat ground: world-frame z is
    // used directly as height-above-ground, no per-frame chassis-height
    // correction. Published as plain PointCloud2 — voxel_slam hands off the
    // raw ROI-filtered points as-is; occupancy-grid rasterization/
    // accumulation is the standalone nav_prob_grid package's job now.
    pcl::PointCloud<PointType> pcl_send_filtered;
    pcl_send_filtered.reserve(pcl_send.size());
    for(PointType &fp: pcl_send)
    {
      if(fp.z >= g_nav_scan_z_min && fp.z <= g_nav_scan_z_max)
        pcl_send_filtered.push_back(fp);
    }
    pub_pl_func(pcl_send_filtered, pub_scan_filtered, x_curr.t);

    // Path point is the chassis-center position, not the raw IMU position,
    // so /map_path stays consistent with the published tf (aft_mapped).
    Eigen::Matrix3d R_chassis;
    Eigen::Vector3d pcurr;
    imu_pose_to_chassis(x_curr.R, x_curr.p, R_chassis, pcurr);

    PointType ap;
    ap.x = pcurr[0];
    ap.y = pcurr[1];
    ap.z = pcurr[2];
    ap.curvature = jour;
    ap.intensity = cur_session;
    pcl_path.push_back(ap);
    pub_pl_func(pcl_path, pub_curr_path);
  }

  void pub_localmap(int mgsize, int cur_session, vector<PVecPtr> &pvec_buf, vector<IMUST> &x_buf, pcl::PointCloud<PointType> &pcl_path, int win_base, int win_count)
  {
    pcl::PointCloud<PointType> pcl_send;
    for(int i=0; i<mgsize; i++)
    {
      for(int j=0; j<pvec_buf[i]->size(); j+=3)
      {
        pointVar &pv = pvec_buf[i]->at(j);
        Eigen::Vector3d pvec = x_buf[i].R*pv.pnt + x_buf[i].p;
        PointType ap;
        ap.x = pvec[0];
        ap.y = pvec[1];
        ap.z = pvec[2];
        ap.intensity = cur_session;
        pcl_send.push_back(ap);
      }
    }

    for(int i=0; i<win_count; i++)
    {
      // Same chassis-center conversion as pub_localtraj, for consistency.
      Eigen::Matrix3d R_chassis;
      Eigen::Vector3d pcurr;
      imu_pose_to_chassis(x_buf[i].R, x_buf[i].p, R_chassis, pcurr);
      pcl_path[i+win_base].x = pcurr[0];
      pcl_path[i+win_base].y = pcurr[1];
      pcl_path[i+win_base].z = pcurr[2];
    }

    pub_pl_func(pcl_path, pub_curr_path);
    pub_pl_func(pcl_send, pub_cmap);
  }

  void pub_global_path(vector<vector<ScanPose*>*> &relc_bl_buf, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub_relc, vector<int> &ids)
  {
    pcl::PointCloud<pcl::PointXYZI> pl;
    pcl::PointXYZI pp;
    int idsize = ids.size();

    for(int i=0; i<idsize; i++)
    {
      pp.intensity = ids[i];
      for(ScanPose* bl: *(relc_bl_buf[ids[i]]))
      {
        // Chassis-center position, matching /map_path's convention.
        Eigen::Matrix3d R_chassis;
        Eigen::Vector3d p_chassis;
        imu_pose_to_chassis(bl->x.R, bl->x.p, R_chassis, p_chassis);
        pp.x = p_chassis[0]; pp.y = p_chassis[1]; pp.z = p_chassis[2];
        pl.push_back(pp);
      }
    }
    pub_pl_func(pl, pub_relc);
  }

  void pub_globalmap(vector<vector<Keyframe*>*> &relc_submaps, vector<int> &ids, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub)
  {
    pcl::PointCloud<pcl::PointXYZI> pl;
    pub_pl_func(pl, pub);
    pcl::PointXYZI pp;

    uint interval_size = 5e6;
    uint psize = 0;
    for(int id: ids)
    {
      vector<Keyframe*> &smps = *(relc_submaps[id]);
      for(int i=0; i<smps.size(); i++)
        psize += smps[i]->plptr->size();
    }
    int jump = psize / (10 * interval_size) + 1;

    for(int id: ids)
    {
      pp.intensity = id;
      vector<Keyframe*> &smps = *(relc_submaps[id]);
      for(int i=0; i<smps.size(); i++)
      {
        IMUST xx = smps[i]->x0;
        for(int j=0; j<smps[i]->plptr->size(); j+=jump)
        // for(int j=0; j<smps[i]->plptr->size(); j+=1)
        {
          PointType &ap = smps[i]->plptr->points[j];
          Eigen::Vector3d vv(ap.x, ap.y, ap.z);
          vv = xx.R * vv + xx.p;
          pp.x = vv[0]; pp.y = vv[1]; pp.z = vv[2];
          pl.push_back(pp);
        }

        if(pl.size() > interval_size)
        {
          pub_pl_func(pl, pub);
          sleep(0.05);
          pl.clear();
        }
      }
    }
    pub_pl_func(pl, pub);
  }

};

class FileReaderWriter
{
public:
  static FileReaderWriter &instance()
  {
    static FileReaderWriter inst;
    return inst;
  }

  // Archives one newly-finalized keyframe's own LOCAL-frame cloud (see
  // Keyframe::plptr's convention — relative to kf.x0, not yet transformed to
  // world) to <savepath><mapname>/kf/<id>.pcd. Called exactly once per
  // keyframe, right where it's kept (not discarded as a redundant near-
  // duplicate) — mirrors publish_keyframe_submap()'s own "send once, on
  // creation" timing, so kf/ ends up holding exactly the same roster
  // nav_prob_grid would have seen live. Only ever called when
  // General.is_save_map && !General.mapname.empty() (see call site).
  //
  // Deliberately NOT paired with a kf/poses.txt (there used to be one —
  // removed): kf.id is just the index of the scan that anchors this
  // keyframe, and alidarState.txt already carries that exact scan's own
  // final pose as one of its rows (see previous_map_read(), which reads
  // the pose from there and the cloud from here) — writing the same pose
  // out a second time, in a second file that would need to be kept in
  // sync across every loop-closure/HBA correction, was a duplicate source
  // of truth for no benefit.
  void save_keyframe_cloud(Keyframe &kf, const string &savepath, const string &mapname)
  {
    string pcdname = savepath + mapname + "/kf/" + to_string(kf.id) + ".pcd";
    pcl::io::savePCDFileBinary(pcdname, *kf.plptr);
  }

  // Appends one "timestamp tx ty tz qx qy qz qw" line to an already-open
  // output stream (lidar_poses.txt / image_poses.txt), same precision
  // convention as save_pose() below (6 decimals for the timestamp, 7 for
  // position/quaternion).
  void append_pose_line(ofstream &f, double ts, const Eigen::Vector3d &p, const Eigen::Matrix3d &R)
  {
    Eigen::Quaterniond qq(R);
    f << fixed << setprecision(6) << ts << " ";
    f << setprecision(7) << p[0] << " " << p[1] << " " << p[2] << " ";
    f << qq.x() << " " << qq.y() << " " << qq.z() << " " << qq.w() << endl;
  }

  void save_pose(vector<ScanPose*> &bbuf, string &fname, string posename, string &savepath)
  {
    if(bbuf.size() < 100) return;
    int topsize = bbuf.size();

    ofstream posfile(savepath + fname + posename);
    for(int i=0; i<topsize; i++)
    {
      IMUST &xx = bbuf[i]->x;
      Eigen::Quaterniond qq(xx.R);
      posfile << fixed << setprecision(6) << xx.t << " ";
      posfile << setprecision(7) << xx.p[0] << " " << xx.p[1] << " " << xx.p[2] << " ";
      posfile << qq.x() << " " << qq.y() << " " << qq.z() << " " << qq.w();
      posfile << " " << xx.v[0] << " " << xx.v[1] << " " << xx.v[2];
      posfile << " " << xx.bg[0] << " " << xx.bg[1] << " " << xx.bg[2];
      posfile << " " << xx.ba[0] << " " << xx.ba[1] << " " << xx.ba[2];
      posfile << " " << xx.g[0] << " " << xx.g[1] << " " << xx.g[2];
      for(int j=0; j<6; j++) posfile << " " << bbuf[i]->v6[j];
      posfile << endl;
    }
    posfile.close();

  }

  // Chassis-center trajectory export, purely for inspection/downstream use.
  // Unlike save_pose()/alidarState.txt, this is never read back by the
  // pipeline (previous_map_read only loads alidarState.txt), so it's safe to
  // re-express every pose in the chassis frame here.
  void save_chassis_traj(vector<ScanPose*> &bbuf, string &fname, string posename, string &savepath)
  {
    if(bbuf.size() < 100) return;
    int topsize = bbuf.size();

    ofstream posfile(savepath + fname + posename);
    for(int i=0; i<topsize; i++)
    {
      IMUST &xx = bbuf[i]->x;
      Eigen::Matrix3d R_chassis;
      Eigen::Vector3d t_chassis;
      imu_pose_to_chassis(xx.R, xx.p, R_chassis, t_chassis);
      Eigen::Quaterniond qq(R_chassis);

      posfile << fixed << setprecision(6) << xx.t << " ";
      posfile << setprecision(7) << t_chassis[0] << " " << t_chassis[1] << " " << t_chassis[2] << " ";
      posfile << qq.x() << " " << qq.y() << " " << qq.z() << " " << qq.w();
      posfile << endl;
    }
    posfile.close();
  }

  // The loop clousure edges of multi sessions
  void pgo_edges_io(PGO_Edges &edges, vector<string> &fnames, int io, string &savepath, string &mapname)
  {
    static vector<string> seq_absent;
    Eigen::Matrix<double, 6, 1> v6_init;
    v6_init << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    if(io == 0) // read
    {
      ifstream infile(savepath + "edge.txt");
      string lineStr, str;
      vector<string> sts;
      while(getline(infile, lineStr))
      {
        sts.clear();
        stringstream ss(lineStr);
        while(ss >> str)
          sts.push_back(str);
        
        int mp[2] = {-1, -1};
        for(int i=0; i<2; i++)
        for(int j=0; j<fnames.size(); j++)
        if(sts[i] == fnames[j])
        {
          mp[i] = j;
          break;
        }

        if(mp[0] != -1 && mp[1] != -1)
        {
          int id1 = stoi(sts[2]);
          int id2 = stoi(sts[3]);
          Eigen::Vector3d v3; 
          v3 << stod(sts[4]), stod(sts[5]), stod(sts[6]);
          Eigen::Quaterniond qq(stod(sts[10]), stod(sts[7]), stod(sts[8]), stod(sts[9]));
          Eigen::Matrix3d rot(qq.matrix());
          if(mp[0] <= mp[1])
            edges.push(mp[0], mp[1], id1, id2, rot, v3, v6_init);
          else
          {
            v3 = -rot.transpose() * v3;
            rot = qq.matrix().transpose();
            edges.push(mp[1], mp[0], id2, id1, rot, v3, v6_init);
          }
        }
        else
        {
          if(sts[0] != mapname && sts[1] != mapname)
            seq_absent.push_back(lineStr);
        }

      }
    }
    else // write
    {
      ofstream outfile(savepath + "edge.txt");
      for(string &str: seq_absent)
        outfile << str << endl;

      for(PGO_Edge &edge: edges.edges)
      {
        for(int i=0; i<edge.rots.size(); i++)
        {
          outfile << fnames[edge.m1] << " ";
          outfile << fnames[edge.m2] << " ";
          outfile << edge.ids1[i] << " ";
          outfile << edge.ids2[i] << " ";
          Eigen::Vector3d v(edge.tras[i]);
          outfile << setprecision(7) << v[0] << " " << v[1] << " " << v[2] << " ";
          Eigen::Quaterniond qq(edge.rots[i]);
          outfile << qq.x() << " " << qq.y() << " " << qq.z() << " " << qq.w() << endl;
        }
      }
      outfile.close();
    }

  }

  // loading the offline map
  void previous_map_names(rclcpp::Node::SharedPtr &n, vector<string> &fnames, vector<double> &juds)
  {
    string premap;
    declare_and_get<string>(n, "General.previous_map", premap, "");
    premap.erase(remove_if(premap.begin(), premap.end(), ::isspace), premap.end());
    stringstream ss(premap);
    string str;
    while(getline(ss, str, ','))
    {
      stringstream ss2(str);
      vector<string> strs;
      while(getline(ss2, str, ':'))
        strs.push_back(str);
      
      if(strs.size() != 2)
      {
        LOG_ERROR(INIT, "previous_map entry malformed | entry:\"{}\" (expected \"name:threshold\")", str);
        return;
      }

      if(strs[0][0] != '#')
      {
        fnames.push_back(strs[0]);
        juds.push_back(stod(strs[1]));
      }
    }

  }

  void previous_map_read(vector<STDescManager*> &std_managers, vector<vector<ScanPose*>*> &multimap_scanPoses, vector<vector<Keyframe*>*> &multimap_keyframes, ConfigSetting &config_setting, PGO_Edges &edges, rclcpp::Node::SharedPtr &n, vector<string> &fnames, vector<double> &juds, string &savepath, int win_size)
  {
    for(int fn=0; fn<fnames.size() && rclcpp::ok(); fn++)
    {
      string fname = savepath + fnames[fn];
      vector<ScanPose*>* bl_tem = new vector<ScanPose*>();
      vector<Keyframe*>* keyframes_tem = new vector<Keyframe*>();
      STDescManager *std_manager = new STDescManager(config_setting);

      std_managers.push_back(std_manager);
      multimap_scanPoses.push_back(bl_tem);
      multimap_keyframes.push_back(keyframes_tem);
      read_lidarstate(fname+"/alidarState.txt", *bl_tem);

      // Only release-owned keyframe archives are supported.
      bool have_kf_archive = (access((fname + "/kf").c_str(), F_OK) == 0);

      if(have_kf_archive)
      {
        // The kf/ directory listing IS the ground truth for "which scan
        // indices actually became a kept keyframe" — a candidate judged
        // redundant during the original live run (Loop.redundant_score_
        // thresh / isRedundant in thd_loop_closure) never got a kf/ entry
        // at all, so there is no fixed "one every win_size scans" pattern
        // to re-derive or guess at here (and no risk of silently
        // misaligning everything if win_size has since been reconfigured)
        // — just read back exactly the ids that exist, in order, and use
        // each one directly.
        vector<int> kf_ids;
        for(const auto &entry: std::filesystem::directory_iterator(fname + "/kf"))
        {
          if(entry.path().extension() != ".pcd") continue;
          try { kf_ids.push_back(stoi(entry.path().stem().string())); }
          catch(...) { continue; } // not a plain <id>.pcd — ignore
        }
        sort(kf_ids.begin(), kf_ids.end());

        LOG_STARTUP(INIT, "Reading {} | scans:{} | source:kf/ archive ({} keyframes)",
                    fname, bl_tem->size(), kf_ids.size());

        for(int id: kf_ids)
        {
          if(!rclcpp::ok()) break;
          if(id < 0 || id >= (int)bl_tem->size())
          {
            LOG_WARN(INIT, "kf/ archive id out of range for this session's own alidarState.txt "
                            "| id:{} scans:{} — skipping", id, bl_tem->size());
            continue;
          }

          Keyframe *smp = new Keyframe(bl_tem->at(id)->x);
          smp->id = id;
          string kf_pcd = fname + "/kf/" + to_string(id) + ".pcd";
          if(pcl::io::loadPCDFile(kf_pcd, *(smp->plptr)) != 0)
          {
            LOG_ERROR(INIT, "kf/ archive listed this file but it failed to load | path:{}", kf_pcd);
            delete smp;
            continue;
          }
          keyframes_tem->push_back(smp);
        }

        if(kf_ids.empty())
          LOG_WARN(INIT, "kf/ exists but has no <id>.pcd entries | session:{}", fname);
      }
      else
        throw std::runtime_error("Missing keyframe archive: " + fname + "/kf");

      // An archived keyframe already contains a complete scan window. Re-merging
      // ten keyframes changed the descriptor input and discarded short sessions.
      pcl::PointCloud<pcl::PointXYZI>::Ptr pl_btc(new pcl::PointCloud<pcl::PointXYZI>());
      size_t descriptor_count = 0;
      for(const Keyframe *kf: *keyframes_tem)
      {
        pcl::copyPointCloud(*kf->plptr, *pl_btc);
        if(pl_btc->empty()) throw std::runtime_error("Empty keyframe in " + fname);
        vector<STD> stds_vec;
        std_manager->GenerateSTDescs(pl_btc, stds_vec, kf->id);
        descriptor_count += stds_vec.size();
        std_manager->AddSTDescs(stds_vec);
      }
      LOG_STARTUP(INIT, "Loaded descriptor database | session:{} keyframes:{} descriptors:{}",
                  fname, keyframes_tem->size(), descriptor_count);
      std_manager->config_setting_.skip_near_num_ = -(std_manager->plane_cloud_vec_.size()+10);

      LOG_STARTUP(INIT, "Read {} done | keyframes:{}", fname, keyframes_tem->size());
    }

    vector<int> ids_all;
    for(int fn=0; fn<fnames.size() && rclcpp::ok(); fn++)
      ids_all.push_back(fn);

    // gtsam::Values initial;
    // gtsam::NonlinearFactorGraph graph;
    // vector<int> ids_cnct, stepsizes;
    // Eigen::Matrix<double, 6, 1> v6_init;
    // v6_init << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    // gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_init));
    // build_graph(initial, graph, ids_all.back(), edges, odom_noise, ids_cnct, stepsizes, 1);

    // gtsam::ISAM2Params parameters;
    // parameters.relinearizeThreshold = 0.01;
    // parameters.relinearizeSkip = 1;
    // gtsam::ISAM2 isam(parameters);
    // isam.update(graph, initial);

    // for(int i=0; i<5; i++) isam.update();
    // gtsam::Values results = isam.calculateEstimate();
    // int resultsize = results.size();
    // int idsize = ids_cnct.size();
    // for(int ii=0; ii<idsize; ii++)
    // {
    //   int tip = ids_cnct[ii];
    //   for(int j=stepsizes[ii]; j<stepsizes[ii+1]; j++)
    //   {
    //     int ord = j - stepsizes[ii];
    //     multimap_scanPoses[tip]->at(ord)->set_state(results.at(j).cast<gtsam::Pose3>());
    //   }
    // }
    // for(int ii=0; ii<idsize; ii++)
    // {
    //   int tip = ids_cnct[ii];
    //   for(Keyframe *kf: *multimap_keyframes[tip])
    //     kf->x0 = multimap_scanPoses[tip]->at(kf->id)->x;
    // }

    bool require_grid = true;
    declare_and_get<bool>(n, "General.require_grid_subscriber", require_grid, true);
    if(require_grid && !fnames.empty())
    {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while(pub_keyframe_submap->get_subscription_count() == 0 && rclcpp::ok() && !g_request_shutdown)
      {
        if(std::chrono::steady_clock::now() > deadline)
          throw std::runtime_error("nav_prob_grid not ready for loaded submap replay");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids_all);
    ResultOutput::instance().pub_globalmap(multimap_keyframes, ids_all, pub_pmap);
    // These keyframes are new to any downstream consumer (nav_prob_grid
    // included) — send full submaps (cloud+pose), not just a pose update.
    for(size_t session_id = 0; session_id < multimap_keyframes.size(); session_id++)
      for(Keyframe *kf: *multimap_keyframes[session_id])
        publish_keyframe_submap((int)session_id, *kf, pub_keyframe_submap);

    LOG_STARTUP(INIT, "All previous maps loaded | sessions:{}", ids_all.size());
  }
  
};

class Initialization
{
public:
  static Initialization &instance()
  {
    static Initialization inst;
    return inst;
  }

  void align_gravity(vector<IMUST> &xs)
  {
    Eigen::Vector3d g0 = xs[0].g;
    Eigen::Vector3d n0 = g0 / g0.norm();
    Eigen::Vector3d n1(0, 0, 1);
    if(n0[2] < 0)
      n1[2] = -1;
    
    Eigen::Vector3d rotvec = n0.cross(n1);
    double rnorm = rotvec.norm();
    rotvec = rotvec / rnorm;

    Eigen::AngleAxisd angaxis(asin(rnorm), rotvec);
    Eigen::Matrix3d rot = angaxis.matrix();
    g0 = rot * g0;

    Eigen::Vector3d p0 = xs[0].p;
    for(int i=0; i<xs.size(); i++)
    {
      xs[i].p = rot * (xs[i].p - p0) + p0;
      xs[i].R = rot * xs[i].R;
      xs[i].v = rot * xs[i].v;
      xs[i].g = g0;
    }

  }

  void motion_blur(pcl::PointCloud<PointType> &pl, PVec &pvec, IMUST xc, IMUST xl, deque<sensor_msgs::msg::Imu::SharedPtr> &imus, double pcl_beg_time, IMUST &extrin_para)
  {
    xc.bg = xl.bg; xc.ba = xl.ba;
    Eigen::Vector3d acc_imu, angvel_avr, acc_avr, vel_imu(xc.v), pos_imu(xc.p);
    Eigen::Matrix3d R_imu(xc.R);
    vector<IMUST> imu_poses;

    for(auto it_imu=imus.end()-1; it_imu!=imus.begin(); it_imu--)
    {
      sensor_msgs::msg::Imu &head = **(it_imu-1);
      sensor_msgs::msg::Imu &tail = **(it_imu); 
      
      angvel_avr << 0.5*(head.angular_velocity.x + tail.angular_velocity.x), 
                    0.5*(head.angular_velocity.y + tail.angular_velocity.y), 
                    0.5*(head.angular_velocity.z + tail.angular_velocity.z);
      acc_avr << 0.5*(head.linear_acceleration.x + tail.linear_acceleration.x), 
                 0.5*(head.linear_acceleration.y + tail.linear_acceleration.y), 
                 0.5*(head.linear_acceleration.z + tail.linear_acceleration.z);

      angvel_avr -= xc.bg;
      acc_avr = acc_avr * imupre_scale_gravity - xc.ba;

      double dt = stamp2sec(head.header.stamp) - stamp2sec(tail.header.stamp);
      Eigen::Matrix3d acc_avr_skew = hat(acc_avr);
      Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);

      acc_imu = R_imu * acc_avr + xc.g;
      pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
      vel_imu = vel_imu + acc_imu * dt;
      R_imu = R_imu * Exp_f;

      double offt = stamp2sec(head.header.stamp) - pcl_beg_time;
      imu_poses.emplace_back(offt, R_imu, pos_imu, vel_imu, angvel_avr, acc_imu);
    }

    pointVar pv; pv.var.setIdentity();
    if(point_notime)
    {
      for(PointType &ap: pl.points)
      {
        pv.pnt << ap.x, ap.y, ap.z;
        pv.pnt = extrin_para.R * pv.pnt + extrin_para.p;
        pvec.push_back(pv);
      }
      return;
    }
    auto it_pcl = pl.end() - 1;
    // for(auto it_kp=imu_poses.end(); it_kp!=imu_poses.begin(); it_kp--)
    for(auto it_kp=imu_poses.begin(); it_kp!=imu_poses.end(); it_kp++)
    {
      // IMUST &head = *(it_kp - 1);
      IMUST &head = *it_kp;
      R_imu = head.R;
      acc_imu = head.ba;
      vel_imu = head.v;
      pos_imu = head.p;
      angvel_avr = head.bg;

      for(; it_pcl->curvature > head.t; it_pcl--)
      {
        double dt = it_pcl->curvature - head.t;
        Eigen::Matrix3d R_i = R_imu * Exp(angvel_avr, dt);
        Eigen::Vector3d T_ei = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - xc.p;

        Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        Eigen::Vector3d P_compensate = xc.R.transpose() * (R_i * (extrin_para.R * P_i + extrin_para.p) + T_ei);

        pv.pnt = P_compensate;
        pvec.push_back(pv);
        if(it_pcl == pl.begin()) break;
      }

    }
  }

  int motion_init(vector<pcl::PointCloud<PointType>::Ptr> &pl_origs, vector<deque<sensor_msgs::msg::Imu::SharedPtr>> &vec_imus, vector<double> &beg_times, Eigen::MatrixXd *hess, LidarFactor &voxhess, vector<IMUST> &x_buf, unordered_map<VOXEL_LOC, OctoTree*> &surf_map, unordered_map<VOXEL_LOC, OctoTree*> &surf_map_slide, vector<PVecPtr> &pvec_buf, int win_size, vector<vector<SlideWindow*>> &sws, IMUST &x_curr, deque<IMU_PRE*> &imu_pre_buf, IMUST &extrin_para)
  {
    PLV(3) pwld;
    double last_g_norm = x_buf[0].g.norm();
    int converge_flag = 0;

    double min_eigen_value_orig = min_eigen_value;
    vector<double> eigen_value_array_orig = plane_eigen_value_thre;

    min_eigen_value = 0.02;
    for(double &iter: plane_eigen_value_thre)
      iter = 1.0 / 4;

    double t0 = rclcpp::Clock().now().seconds();
    double converge_thre = 0.05;
    int converge_times = 0;
    bool is_degrade = true;
    Eigen::Vector3d eigvalue; eigvalue.setZero();
    for(int iterCnt = 0; iterCnt < 10; iterCnt++)
    {
      if(converge_flag == 1)
      {
        min_eigen_value = min_eigen_value_orig;
        plane_eigen_value_thre = eigen_value_array_orig;
      }

      vector<OctoTree*> octos;
      for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      {
        iter->second->tras_ptr(octos);
        iter->second->clear_slwd(sws[0]);
        delete iter->second;
      }
      for(int i=0; i<octos.size(); i++)
        delete octos[i];
      surf_map.clear(); octos.clear(); surf_map_slide.clear();

      for(int i=0; i<win_size; i++)
      {
        pwld.clear();
        pvec_buf[i]->clear();
        int l = i==0 ? i : i - 1;
        motion_blur(*pl_origs[i], *pvec_buf[i], x_buf[i], x_buf[l], vec_imus[i], beg_times[i], extrin_para);

        if(converge_flag == 1)
        {
          for(pointVar &pv: *pvec_buf[i])
            calcBodyVar(pv.pnt, dept_err, beam_err, pv.var);
          pvec_update(pvec_buf[i], x_buf[i], pwld);
        }
        else
        {
          for(pointVar &pv: *pvec_buf[i])
            pwld.push_back(x_buf[i].R * pv.pnt + x_buf[i].p);
        }

        cut_voxel(surf_map, pvec_buf[i], i, surf_map_slide, win_size, pwld, sws[0]);
      }

      // LidarFactor voxhess(win_size);
      voxhess.clear(); voxhess.win_size = win_size;
      for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      {
        iter->second->recut(win_size, x_buf, sws[0]);
        iter->second->tras_opt(voxhess);
      }

      if(voxhess.plvec_voxels.size() < 10)
        break;
      LI_BA_OptimizerGravity opt_lsv;
      vector<double> resis;
      opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, resis, hess, 3);
      Eigen::Matrix3d nnt; nnt.setZero();

      LOG_DEBUG(INIT, "gravity iter {} | g:({:.4f} {:.4f} {:.4f}) |g|:{:.4f} | resi_delta_ratio:{:.6f}",
                iterCnt, x_buf[0].g[0], x_buf[0].g[1], x_buf[0].g[2], x_buf[0].g.norm(), fabs(resis[0] - resis[1]) / resis[0]);

      for(int i=0; i<win_size-1; i++)
        delete imu_pre_buf[i];
      imu_pre_buf.clear();

      for(int i=1; i<win_size; i++)
      {
        imu_pre_buf.push_back(new IMU_PRE(x_buf[i-1].bg, x_buf[i-1].ba));
        imu_pre_buf.back()->push_imu(vec_imus[i]);
      }

      if(fabs(resis[0] - resis[1]) / resis[0] < converge_thre && iterCnt >= 2)
      {
        for(Eigen::Matrix3d &iter: voxhess.eig_vectors)
        {
          Eigen::Vector3d v3 = iter.col(0);
          nnt += v3 * v3.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
        eigvalue = saes.eigenvalues();
        is_degrade = eigvalue[0] < 15 ? true : false;

        converge_thre = 0.01;
        if(converge_flag == 0)
        {
          align_gravity(x_buf);
          converge_flag = 1;
          continue;
        }
        else
          break;
      }
    }

    x_curr = x_buf[win_size - 1];
    double gnm = x_curr.g.norm();
    if(is_degrade || gnm < 9.6 || gnm > 10.0)
    {
      converge_flag = 0;
    }
    if(converge_flag == 0)
    {
      vector<OctoTree*> octos;
      for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      {
        iter->second->tras_ptr(octos);
        iter->second->clear_slwd(sws[0]);
        delete iter->second;
      }
      for(int i=0; i<octos.size(); i++)
        delete octos[i];
      surf_map.clear(); octos.clear(); surf_map_slide.clear();
    }

    LOG_STARTUP(INIT, "gravity init done | eigvalues:({:.4f} {:.4f} {:.4f}) | degrade_thresh:15 | is_degrade:{}",
             eigvalue[0], eigvalue[1], eigvalue[2], is_degrade);
    Eigen::Vector3d angv(vec_imus[0][0]->angular_velocity.x, vec_imus[0][0]->angular_velocity.y, vec_imus[0][0]->angular_velocity.z);
    Eigen::Vector3d acc(vec_imus[0][0]->linear_acceleration.x, vec_imus[0][0]->linear_acceleration.y, vec_imus[0][0]->linear_acceleration.z);
    acc *= 9.8;

    pl_origs.clear(); vec_imus.clear(); beg_times.clear();
    double t1 = rclcpp::Clock().now().seconds();
    LOG_INFO(PERF, "gravity init | cost:{:.1f}ms", (t1 - t0) * 1000.0);

    // align_gravity(x_buf);
    pcl::PointCloud<PointType> pcl_send; PointType pt;
    for(int i=0; i<win_size; i++)
    for(pointVar &pv: *pvec_buf[i])
    {
      Eigen::Vector3d vv = x_buf[i].R * pv.pnt + x_buf[i].p;
      pt.x = vv[0]; pt.y = vv[1]; pt.z = vv[2];
      pcl_send.push_back(pt);
    }
    pub_pl_func(pcl_send, pub_init);

    return converge_flag;
  }

};

class VOXEL_SLAM
{
public:
  pcl::PointCloud<PointType> pcl_path;
  IMUST x_curr, extrin_para;
  IMUEKF odom_ekf;
  unordered_map<VOXEL_LOC, OctoTree*> surf_map, surf_map_slide;
  // LRU recency list for surf_map, front=most-recently-used, back=least. Kept
  // in sync with surf_map at every insertion/hit (via cut_voxel*'s lru_touch)
  // and at every place surf_map is bulk-rebuilt or has an entry erased.
  // surf_map_capacity<=0 disables capacity eviction (list is still maintained,
  // just never drained), matching the default in config so existing bags/
  // deployments keep today's jour-only eviction behavior unless opted in.
  std::list<VOXEL_LOC> surf_map_lru;
  int surf_map_capacity = 0;
  double down_size;

  int win_size;
  vector<IMUST> x_buf;
  vector<PVecPtr> pvec_buf;
  deque<IMU_PRE*> imu_pre_buf;
  int win_count = 0, win_base = 0;
  vector<vector<SlideWindow*>> sws;

  vector<ScanPose*> *scanPoses;
  mutex mtx_loop;
  deque<ScanPose*> buf_lba2loop, buf_lba2loop_tem;
  vector<Keyframe*> *keyframes;
  std::atomic<int> loop_detect{0};
  std::atomic<bool> registration_pending{false};
  unordered_map<VOXEL_LOC, OctoTree*> map_loop;
  IMUST dx;
  pcl::PointCloud<PointType>::Ptr pl_kdmap;
  pcl::KdTreeFLANN<PointType> kd_keyframes;
  int history_kfsize = 0;
  vector<OctoTree*> octos_release;
  int reset_flag = 0;
  int g_update = 0;
  int thread_num = 5;
  int degrade_bound = 10;
  // Physical motion limits of the chassis (omnidirectional base): used only
  // to flag a suspicious pose jump between consecutive scans, not to clamp
  // or reject anything. See the per-scan check in thd_odometry_localmapping.
  double pose_jump_max_lin_vel = 0.5;   // m/s per axis
  double pose_jump_max_ang_vel = 0.6;   // rad/s

  vector<vector<ScanPose*>*> multimap_scanPoses;
  vector<vector<Keyframe*>*> multimap_keyframes;
  std::atomic<int> gba_flag{0};
  int gba_size = 0;
  vector<int> cnct_map;
  mutex mtx_keyframe;
  PGO_Edges gba_edges1, gba_edges2;
  std::atomic<bool> is_finish{false};

  vector<string> sessionNames;
  string mapname, savepath;
  int is_save_map;
  bool image_feature_enabled = false;
  ofstream lidar_pose_ofs, image_pose_ofs;

  // (Re)opens lidar_poses.txt (always, when is_save_map) and image_poses.txt
  // (only when image_feature_enabled) truncated in savepath+mapname/. Called
  // from the constructor and again from system_reset() whenever mapname
  // changes to a new session folder.
  void open_output_streams()
  {
    if(lidar_pose_ofs.is_open()) lidar_pose_ofs.close();
    if(image_pose_ofs.is_open()) image_pose_ofs.close();

    if(!is_save_map) return;

    lidar_pose_ofs.open(savepath + mapname + "/lidar_poses.txt", ios::trunc);
    if(image_feature_enabled)
      image_pose_ofs.open(savepath + mapname + "/image_poses.txt", ios::trunc);
  }

  VOXEL_SLAM(rclcpp::Node::SharedPtr &n)
  {
    double cov_gyr, cov_acc, rand_walk_gyr, rand_walk_acc;
    vector<double> vecR(9), vecT(3);
    scanPoses = new vector<ScanPose*>();
    keyframes = new vector<Keyframe*>();
    
    string lid_topic, imu_topic;
    declare_and_get<string>(n, "General.lid_topic", lid_topic, "/livox/lidar");
    declare_and_get<string>(n, "General.imu_topic", imu_topic, "/livox/imu");
    declare_and_get<string>(n, "General.mapname", mapname, "site3_handheld_4");
    declare_and_get<string>(n, "General.save_path", savepath, "");
    declare_and_get<int>(n, "General.lidar_type", feat.lidar_type, 0);
    // Chassis-centered blind-zone box (see Features::point_in_blind() in
    // feature_point.hpp) — blind_half_x/y are half-extents along the
    // chassis's own forward/left axes (half the actual chassis footprint's
    // depth/width, so the box just contains it — see the rotation set up
    // for feat.blind_R below); blind_z_min/max are heights relative to
    // the chassis origin (blind_center), NOT the ground — defaults here
    // are ground 0 and robot-top 1.70m, shifted down by the same 0.096m
    // chassis-center-above-ground offset used for nav_scan_z_min/max
    // below.
    declare_and_get<double>(n, "General.blind_half_x", feat.blind_half_x, 0.288);
    declare_and_get<double>(n, "General.blind_half_y", feat.blind_half_y, 0.2945);
    declare_and_get<double>(n, "General.blind_z_min", feat.blind_z_min, -0.096);
    declare_and_get<double>(n, "General.blind_z_max", feat.blind_z_max, 1.604);
    declare_and_get<int>(n, "General.point_filter_num", feat.point_filter_num, 3);
    declare_and_get<vector<double>>(n, "General.extrinsic_tran", vecT, vector<double>());
    declare_and_get<vector<double>>(n, "General.extrinsic_rota", vecR, vector<double>());
    declare_and_get<int>(n, "General.is_save_map", is_save_map, 0);

    // Optional RGB-D color image sink (General.image_topic). Empty (the
    // default) disables the feature entirely — no subscription, no
    // images/ dir, no image_poses.txt.
    string image_topic;
    declare_and_get<string>(n, "General.image_topic", image_topic, "");
    image_feature_enabled = !image_topic.empty();
    if(image_feature_enabled)
    {
      vector<double> vecR_il(9), vecT_il(3);
      declare_and_get<vector<double>>(n, "General.image_extrinsic_rota", vecR_il, vector<double>());
      declare_and_get<vector<double>>(n, "General.image_extrinsic_tran", vecT_il, vector<double>());
      if(vecR_il.size() == 9 && vecT_il.size() == 3)
      {
        g_R_cam_lidar << vecR_il[0], vecR_il[1], vecR_il[2],
                          vecR_il[3], vecR_il[4], vecR_il[5],
                          vecR_il[6], vecR_il[7], vecR_il[8];
        g_t_cam_lidar << vecT_il[0], vecT_il[1], vecT_il[2];
      }
      else
        LOG_ERROR(CAMERA, "General.image_extrinsic_rota/tran malformed (need 9/3 values) — "
                           "left untransformed (identity); unused for image_poses.txt anyway");

      double image_time_tolerance_ms;
      declare_and_get<double>(n, "General.image_time_tolerance_ms", image_time_tolerance_ms, 50.0);
      g_image_time_tol_sec = image_time_tolerance_ms / 1000.0;

      sub_image = n->create_subscription<sensor_msgs::msg::CompressedImage>(image_topic, 200,
        [](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg){ image_handler(msg); });

      LOG_STARTUP(CAMERA, "image_topic enabled | topic:{} tolerance_ms:{:.1f}", image_topic, image_time_tolerance_ms);
    }

    // See g_imu_cbg's comment (voxelslam.hpp) — this subscription is
    // serviced by its own dedicated executor/thread, not by
    // thd_odometry_localmapping's spin_some(n), so it keeps draining IMU
    // messages (and feeding HighRateOdom::on_imu()) even while that thread
    // is busy inside a scan's optimization.
    rclcpp::SubscriptionOptions imu_sub_opts;
    imu_sub_opts.callback_group = g_imu_cbg;
    sub_imu = n->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 80000, imu_handler, imu_sub_opts);
    int lidar_queue_depth;
    declare_and_get<int>(n, "General.lidar_queue_depth", lidar_queue_depth, 1000);
    if(lidar_queue_depth < 1 || lidar_queue_depth > 1000)
      throw std::invalid_argument("General.lidar_queue_depth must be in [1,1000]");
    int pending_scan_limit;
    declare_and_get<int>(n, "General.pending_scan_limit", pending_scan_limit, 0);
    if(pending_scan_limit < 0 || pending_scan_limit > 1000)
      throw std::invalid_argument("General.pending_scan_limit must be in [0,1000]");
    lidar_pending_depth = static_cast<size_t>(pending_scan_limit);
    sub_pcl_pc2 = n->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, lidar_queue_depth,
      [](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){ pcl_handler(msg); });
    odom_ekf.imu_topic = imu_topic;

    // Optional second lidar ("lidar_back"). When General.lid_topic_back is
    // non-empty, its scans (same General.lidar_type/blind/point_filter_num as
    // the front lidar) are re-expressed into the front lidar's frame via
    // General.back_extrinsic_{tran,rota} (p_front = R*p_back + T; default
    // below is the factory calibration for this rig's back lidar) and fed
    // into the same pcl_buf/time_buf stream through pcl_handler_back().
    string lid_topic_back;
    declare_and_get<string>(n, "General.lid_topic_back", lid_topic_back, "");
    if(!lid_topic_back.empty())
    {
      vector<double> vecR_bf(9), vecT_bf(3);
      declare_and_get<vector<double>>(n, "General.back_extrinsic_rota", vecR_bf,
        vector<double>({ 0.9993832234,  0.0345138058, -0.0064784208,
                         -0.0345206446,  0.9994035365, -0.0009467439,
                          0.0064418809,  0.0011697992,  0.9999785666}));
      declare_and_get<vector<double>>(n, "General.back_extrinsic_tran", vecT_bf,
        vector<double>({0.0049381117, 0.0012625173, -0.0043607170}));

      if(vecR_bf.size() == 9 && vecT_bf.size() == 3)
      {
        g_R_back_front << vecR_bf[0], vecR_bf[1], vecR_bf[2],
                           vecR_bf[3], vecR_bf[4], vecR_bf[5],
                           vecR_bf[6], vecR_bf[7], vecR_bf[8];
        g_t_back_front << vecT_bf[0], vecT_bf[1], vecT_bf[2];
      }
      else
        LOG_ERROR(LIDAR, "General.back_extrinsic_rota/tran malformed (need 9/3 values) — "
                          "lidar_back left untransformed (identity)");

      sub_pcl_pc2_back = n->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic_back, lidar_queue_depth,
        [](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){ pcl_handler_back(msg); });

      // Hardware-sync tolerance for pairing a front scan with a back scan
      // (see try_pair_and_push() in voxelslam.hpp) — the two lidars are
      // triggered together, so their header stamps should only differ by
      // driver/network jitter; default is half of a typical 10Hz scan period.
      double back_sync_tolerance_ms;
      declare_and_get<double>(n, "General.back_sync_tolerance_ms", back_sync_tolerance_ms, 50.0);
      g_back_sync_tol_sec = back_sync_tolerance_ms / 1000.0;
      g_dual_lidar_enabled = true;

      LOG_STARTUP(LIDAR, "lidar_back enabled | topic:{} sync_tolerance_ms:{:.1f}", lid_topic_back, back_sync_tolerance_ms);
    }

    declare_and_get<double>(n, "Odometry.cov_gyr", cov_gyr, 0.1);
    declare_and_get<double>(n, "Odometry.cov_acc", cov_acc, 0.1);
    declare_and_get<double>(n, "Odometry.rdw_gyr", rand_walk_gyr, 1e-4);
    declare_and_get<double>(n, "Odometry.rdw_acc", rand_walk_acc, 1e-4);
    declare_and_get<double>(n, "Odometry.down_size", down_size, 0.1);
    declare_and_get<double>(n, "Odometry.dept_err", dept_err, 0.02);
    declare_and_get<double>(n, "Odometry.beam_err", beam_err, 0.05);
    declare_and_get<double>(n, "Odometry.voxel_size", voxel_size, 1);
    declare_and_get<double>(n, "Odometry.min_eigen_value", min_eigen_value, 0.0025);
    declare_and_get<int>(n, "Odometry.degrade_bound", degrade_bound, 10);
    declare_and_get<double>(n, "Odometry.pose_jump_max_lin_vel", pose_jump_max_lin_vel, 0.5);
    declare_and_get<double>(n, "Odometry.pose_jump_max_ang_vel", pose_jump_max_ang_vel, 0.6);
    declare_and_get<int>(n, "Odometry.point_notime", point_notime, 0);
    odom_ekf.point_notime = point_notime;
    // <=0 disables capacity-based eviction; only the pre-existing jour-based
    // (distance-since-touch) eviction runs, matching pre-LRU behavior.
    declare_and_get<int>(n, "Map.surf_map_capacity", surf_map_capacity, 0);

    odom_ekf.cov_gyr << cov_gyr, cov_gyr, cov_gyr;
    odom_ekf.cov_acc << cov_acc, cov_acc, cov_acc;
    odom_ekf.cov_bias_gyr << rand_walk_gyr, rand_walk_gyr, rand_walk_gyr;
    odom_ekf.cov_bias_acc << rand_walk_acc, rand_walk_acc, rand_walk_acc;
    odom_ekf.Lid_offset_to_IMU  << vecT[0], vecT[1], vecT[2];
    odom_ekf.Lid_rot_to_IMU << vecR[0], vecR[1], vecR[2],
                            vecR[3], vecR[4], vecR[5],
                            vecR[6], vecR[7], vecR[8];                
    extrin_para.R = odom_ekf.Lid_rot_to_IMU;
    extrin_para.p = odom_ekf.Lid_offset_to_IMU;
    min_point << 5, 5, 5, 5;

    // Optional lidar_front->chassis extrinsic (parent_frame: chassis, same
    // p_chassis = R*p_lidar + T convention as extrinsic_tran/rota above).
    // Combined with the lidar->IMU extrinsic already loaded, this gives the
    // fixed IMU->chassis offset used purely to re-express solved IMU poses as
    // chassis-center poses for output (see imu_pose_to_chassis()). Left as
    // identity/zero (no-op) when not provided, so existing configs without a
    // chassis extrinsic are unaffected.
    vector<double> vecR_cl(9), vecT_cl(3);
    declare_and_get<vector<double>>(n, "General.chassis_extrinsic_rota", vecR_cl, vector<double>());
    declare_and_get<vector<double>>(n, "General.chassis_extrinsic_tran", vecT_cl, vector<double>());
    if(vecR_cl.size() == 9 && vecT_cl.size() == 3)
    {
      Eigen::Matrix3d R_chassis_lidar;
      R_chassis_lidar << vecR_cl[0], vecR_cl[1], vecR_cl[2],
                          vecR_cl[3], vecR_cl[4], vecR_cl[5],
                          vecR_cl[6], vecR_cl[7], vecR_cl[8];
      Eigen::Vector3d t_chassis_lidar(vecT_cl[0], vecT_cl[1], vecT_cl[2]);

      // T_lidar_chassis = inverse(T_chassis_lidar)
      Eigen::Matrix3d R_lidar_chassis = R_chassis_lidar.transpose();
      Eigen::Vector3d t_lidar_chassis = -R_lidar_chassis * t_chassis_lidar;

      // T_imu_chassis = T_imu_lidar * T_lidar_chassis
      g_R_imu_chassis = extrin_para.R * R_lidar_chassis;
      g_t_imu_chassis = extrin_para.R * t_lidar_chassis + extrin_para.p;

      // T_chassis_imu = inverse(T_imu_chassis) — becomes the odometry's
      // genesis pose below, so the whole world frame is chassis-anchored.
      g_R_chassis_imu = g_R_imu_chassis.transpose();
      g_t_chassis_imu = -g_R_chassis_imu * g_t_imu_chassis;

      // Chassis origin expressed in each lidar's own raw frame, for the
      // blind-zone check in feature_point.hpp (see g_chassis_in_lidar_front/
      // back's comment). t_lidar_chassis is already exactly that for the
      // front lidar; for the back lidar, undo the back->front extrinsic on
      // the front-frame offset.
      g_chassis_in_lidar_front = t_lidar_chassis;
      g_chassis_in_lidar_back = g_R_back_front.transpose() * (t_lidar_chassis - g_t_back_front);
      // Rotation from each lidar's own raw axes into the chassis's own
      // aligned axes, for the same box-shaped blind-zone check (see
      // g_chassis_R_in_lidar_front/back's comment) — R_chassis_lidar (the
      // un-transposed one, already used above for T_imu_chassis) is
      // already exactly that for the front lidar; for the back lidar,
      // compose with the back->front rotation first.
      g_chassis_R_in_lidar_front = R_chassis_lidar;
      g_chassis_R_in_lidar_back = R_chassis_lidar * g_R_back_front;
    }

    // Genesis pose: with no chassis extrinsic configured this is R=I, p=0
    // (identical to prior behavior). With one configured, this anchors the
    // world frame at the chassis's start pose instead of the IMU's — see
    // system_reset() and ekf_imu.hpp's gravity init for the matching pieces.
    x_curr.R = g_R_chassis_imu;
    x_curr.p = g_t_chassis_imu;

    // Height ROI for the navigation-facing filtered scan (/map_scan_filtered,
    // see pub_localtraj). Assumes flat ground: world z is used directly as
    // height-above-ground.
    declare_and_get<double>(n, "General.nav_scan_z_min", g_nav_scan_z_min, 0.05);
    declare_and_get<double>(n, "General.nav_scan_z_max", g_nav_scan_z_max, 1.63);

    declare_and_get<int>(n, "LocalBA.win_size", win_size, 10);
    declare_and_get<int>(n, "LocalBA.max_layer", max_layer, 2);
    declare_and_get<double>(n, "LocalBA.cov_gyr", cov_gyr, 0.1);
    declare_and_get<double>(n, "LocalBA.cov_acc", cov_acc, 0.1);
    declare_and_get<double>(n, "LocalBA.rdw_gyr", rand_walk_gyr, 1e-4);
    declare_and_get<double>(n, "LocalBA.rdw_acc", rand_walk_acc, 1e-4);
    declare_and_get<int>(n, "LocalBA.min_ba_point", min_ba_point, 20);
    declare_and_get<vector<double>>(n, "LocalBA.plane_eigen_value_thre", plane_eigen_value_thre, vector<double>({1, 1, 1, 1}));
    declare_and_get<double>(n, "LocalBA.imu_coef", imu_coef, 1e-4);
    declare_and_get<int>(n, "LocalBA.thread_num", thread_num, 5);

    for(double &iter: plane_eigen_value_thre) iter = 1.0 / iter;
    // for(double &iter: plane_eigen_value_thre) iter = 1.0 / iter;

    noiseMeas.setZero(); noiseWalk.setZero();
    noiseMeas.diagonal() << cov_gyr, cov_gyr, cov_gyr, 
                            cov_acc, cov_acc, cov_acc;
    noiseWalk.diagonal() << 
    rand_walk_gyr, rand_walk_gyr, rand_walk_gyr, 
    rand_walk_acc, rand_walk_acc, rand_walk_acc;

    int ss = 0;
    if(access((savepath+mapname+"/").c_str(), X_OK) == -1)
    {
      string cmd = "mkdir " + savepath + mapname + "/";
      ss = system(cmd.c_str());
    }
    else
      ss = -1;

    if(ss != 0 && is_save_map == 1)
    {
      LOG_ERROR(INIT, "session folder collision | path:{} | is_save_map:1 | cause:{} — refusing to overwrite, clear or rename it",
                savepath + mapname + "/", ss == -1 ? "already_exists" : "mkdir_failed");
      exit(0);
    }

    if(is_save_map && image_feature_enabled)
      system(("mkdir -p " + savepath + mapname + "/images/").c_str());
    // kf/ archive (this session's own keyframe clouds, see
    // save_keyframe_cloud) — requires a real map name, same guard as every
    // kf/ write below.
    if(is_save_map && !mapname.empty())
      system(("mkdir -p " + savepath + mapname + "/kf/").c_str());
    open_output_streams();

    sws.resize(thread_num);
    LOG_STARTUP(INIT, "mapname:{}", mapname);
  }

  // The point-to-plane alignment for odometry
  bool lio_state_estimation(PVecPtr pptr)
  {
    IMUST x_prop = x_curr;

    const int num_max_iter = 4;
    bool EKF_stop_flg = 0, flg_EKF_converged = 0;
    Eigen::Matrix<double, DIM, DIM> G, H_T_H, I_STATE;
    G.setZero(); H_T_H.setZero(); I_STATE.setIdentity();
    int rematch_num = 0;
    int match_num = 0;

    int psize = pptr->size();
    vector<OctoTree*> octos;
    octos.resize(psize, nullptr);

    Eigen::Matrix3d nnt; 
    Eigen::Matrix<double, DIM, DIM> cov_inv = x_curr.cov.inverse();
    for(int iterCount=0; iterCount<num_max_iter; iterCount++)
    {
      Eigen::Matrix<double, 6, 6> HTH; HTH.setZero();
      Eigen::Matrix<double, 6, 1> HTz; HTz.setZero();
      Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);
      Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);
      match_num = 0;
      nnt.setZero();

      for(int i=0; i<psize; i++)
      {
        pointVar &pv = pptr->at(i);
        Eigen::Matrix3d phat = hat(pv.pnt);
        Eigen::Matrix3d var_world = x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
        Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;

        double sigma_d = 0;
        Plane* pla = nullptr;
        int flag = 0;
        if(octos[i] != nullptr && octos[i]->inside(wld))
        {
          double max_prob = 0;
          flag = octos[i]->match(wld, pla, max_prob, var_world, sigma_d, octos[i]);
        }
        else
        {
          flag = match(surf_map, wld, pla, var_world, sigma_d, octos[i]);
        }

        if(flag)
        // if(pla != nullptr)
        {
          Plane &pp = *pla;
          double R_inv = 1.0 / (0.0005 + sigma_d);
          double resi = pp.normal.dot(wld - pp.center);

          Eigen::Matrix<double, 6, 1> jac;
          jac.head(3) = phat * x_curr.R.transpose() * pp.normal;
          jac.tail(3) = pp.normal;
          HTH += R_inv * jac * jac.transpose();
          HTz -= R_inv * jac * resi;
          nnt += pp.normal * pp.normal.transpose();
          match_num++;
        }

      }

      H_T_H.block<6, 6>(0, 0) = HTH;
      Eigen::Matrix<double, DIM, DIM> K_1 = (H_T_H + cov_inv).inverse();
      G.block<DIM, 6>(0, 0) = K_1.block<DIM, 6>(0, 0) * HTH;
      Eigen::Matrix<double, DIM, 1> vec = x_prop - x_curr;
      Eigen::Matrix<double, DIM, 1> solution = K_1.block<DIM, 6>(0, 0) * HTz + vec - G.block<DIM, 6>(0, 0) * vec.block<6, 1>(0, 0);

      x_curr += solution;
      Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
      Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

      EKF_stop_flg = false;
      flg_EKF_converged = false;

      if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015)) 
        flg_EKF_converged = true;

      if(flg_EKF_converged || ((rematch_num==0) && (iterCount==num_max_iter-2)))
      {       
        rematch_num++;
      }

      if(rematch_num >= 2 || (iterCount == num_max_iter-1))
      {
        x_curr.cov = (I_STATE - G) * x_curr.cov;
        EKF_stop_flg = true;
      }

      if(EKF_stop_flg) break;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
    Eigen::Vector3d evalue = saes.eigenvalues();
    // printf("eva %d: %lf\n", match_num, evalue[0]);

    if(evalue[0] < 14)
    {
      LOG_WARN_THROTTLE(EKF, 50, "lio_state_estimation degraded | evalue0:{:.4f} thresh:14", evalue[0]);
      return false;
    }
    else
      return true;
  }

  // The point-to-plane alignment for initialization
  pcl::PointCloud<PointType>::Ptr pl_tree;
  void lio_state_estimation_kdtree(PVecPtr pptr)
  {
    static pcl::KdTreeFLANN<PointType> kd_map;
    if(pl_tree->size() < 100)
    {
      for(pointVar pv: *pptr)
      {
        PointType pp;
        pv.pnt = x_curr.R * pv.pnt + x_curr.p;
        pp.x = pv.pnt[0]; pp.y = pv.pnt[1]; pp.z = pv.pnt[2];
        pl_tree->push_back(pp);
      }
      kd_map.setInputCloud(pl_tree);
      return;
    }

    const int num_max_iter = 4;
    IMUST x_prop = x_curr;
    int psize = pptr->size();
    bool EKF_stop_flg = 0, flg_EKF_converged = 0;
    Eigen::Matrix<double, DIM, DIM> G, H_T_H, I_STATE;
    G.setZero(); H_T_H.setZero(); I_STATE.setIdentity();

    double max_dis = 2*2;
    vector<float> sqdis(NMATCH); vector<int> nearInd(NMATCH);
    PLV(3) vecs(NMATCH);
    int rematch_num = 0;
    Eigen::Matrix<double, DIM, DIM> cov_inv = x_curr.cov.inverse();

    Eigen::Matrix<double, NMATCH, 1> b;
    b.setOnes();
    b *= -1.0f;

    vector<double> ds(psize, -1);
    PLV(3) directs(psize);
    bool refind = true;

    for(int iterCount=0; iterCount<num_max_iter; iterCount++)
    {
      Eigen::Matrix<double, 6, 6> HTH; HTH.setZero();
      Eigen::Matrix<double, 6, 1> HTz; HTz.setZero();
      int valid = 0;
      for(int i=0; i<psize; i++)
      {
        pointVar &pv = pptr->at(i);
        Eigen::Matrix3d phat = hat(pv.pnt);
        Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;

        if(refind)
        {
          PointType apx;
          apx.x = wld[0]; apx.y = wld[1]; apx.z = wld[2];
          kd_map.nearestKSearch(apx, NMATCH, nearInd, sqdis);

          Eigen::Matrix<double, NMATCH, 3> A;
          for(int i=0; i<NMATCH; i++)
          {
            PointType &pp = pl_tree->points[nearInd[i]];
            A.row(i) << pp.x, pp.y, pp.z;
          }
          Eigen::Vector3d direct = A.colPivHouseholderQr().solve(b);
          bool check_flag = false;
          for(int i=0; i<NMATCH; i++)
          {
            if(fabs(direct.dot(A.row(i)) + 1.0) > 0.1) 
              check_flag = true;
          }

          if(check_flag) 
          {
            ds[i] = -1;
            continue;
          }
          
          double d = 1.0 / direct.norm();
          // direct *= d;
          ds[i] = d;
          directs[i] = direct * d;
        }

        if(ds[i] >= 0)
        {
          double pd2 = directs[i].dot(wld) + ds[i];
          Eigen::Matrix<double, 6, 1> jac_s;
          jac_s.head(3) = phat * x_curr.R.transpose() * directs[i];
          jac_s.tail(3) = directs[i];

          HTH += jac_s * jac_s.transpose();
          HTz += jac_s * (-pd2);
          valid++;
        }
      }

      H_T_H.block<6, 6>(0, 0) = HTH;
      Eigen::Matrix<double, DIM, DIM> K_1 = (H_T_H + cov_inv / 1000).inverse();
      G.block<DIM, 6>(0, 0) = K_1.block<DIM, 6>(0, 0) * HTH;
      Eigen::Matrix<double, DIM, 1> vec = x_prop - x_curr;
      Eigen::Matrix<double, DIM, 1> solution = K_1.block<DIM, 6>(0, 0) * HTz + vec - G.block<DIM, 6>(0, 0) * vec.block<6, 1>(0, 0);

      x_curr += solution;
      Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
      Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

      refind = false;
      if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015))
      {
        refind = true;
        flg_EKF_converged = true;
        rematch_num++;
      }

      if(iterCount == num_max_iter-2 && !flg_EKF_converged)
      {
        refind = true;
      }

      if(rematch_num >= 2 || (iterCount == num_max_iter-1))
      {
        x_curr.cov = (I_STATE - G) * x_curr.cov;
        EKF_stop_flg = true;
      }

      if(EKF_stop_flg) break;
    }

    double tt1 = rclcpp::Clock().now().seconds();
    for(pointVar pv: *pptr)
    {
      pv.pnt = x_curr.R * pv.pnt + x_curr.p;
      PointType ap;
      ap.x = pv.pnt[0]; ap.y = pv.pnt[1]; ap.z = pv.pnt[2];
      pl_tree->push_back(ap);
    }
    down_sampling_voxel(*pl_tree, 0.5);
    kd_map.setInputCloud(pl_tree);
    double tt2 = rclcpp::Clock().now().seconds();
  }

  // After detecting loop closure, refine current map and states
  void loop_update()
  {
    LOG_INFO(EKF, "loop correction applied | dR_applied:1 | slide_window:{}", sws[0].size());
    // Snapshot the current frame's chassis-center pose before the loop
    // correction (dx) is applied below, so the before/after change can be
    // logged for comparison once the correction has propagated through
    // x_buf/x_curr. Chassis frame, not raw IMU, to match pub_localtraj/
    // pub_localmap/save_chassis_traj's convention.
    Eigen::Matrix3d R_chassis_before;
    Eigen::Vector3d p_before_loop;
    imu_pose_to_chassis(x_curr.R, x_curr.p, R_chassis_before, p_before_loop);
    Eigen::Quaterniond q_before_loop(R_chassis_before);
    double t1 = rclcpp::Clock().now().seconds();
    for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
    {
      // octos_release.push_back(iter->second);
      iter->second->tras_ptr(octos_release);
      iter->second->clear_slwd(sws[0]);
      delete iter->second; iter->second = nullptr;
    }
    surf_map.clear(); surf_map_slide.clear();
    surf_map_lru.clear();
    surf_map = map_loop;
    map_loop.clear();
    // map_loop's voxels were never touched with surf_map's lru_list (it's a
    // separate map not subject to capacity eviction), so rebuild recency
    // from scratch here, treating every adopted voxel as freshly touched.
    for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
    {
      surf_map_lru.push_front(iter->first);
      iter->second->lru_it = surf_map_lru.begin();
      iter->second->has_lru = true;
    }

    LOG_DEBUG(EKF, "loop_update state | scanPoses:{} buf_lba2loop:{} x_buf:{} win_base:{} win_count:{} slide_window:{}",
              scanPoses->size(), buf_lba2loop.size(), x_buf.size(), win_base, win_count, sws[0].size());
    int blsize = scanPoses->size();
    PointType ap = pcl_path[0];
    pcl_path.clear();
    
    for(int i=0; i<blsize; i++)
    {
      ap.x = scanPoses->at(i)->x.p[0];
      ap.y = scanPoses->at(i)->x.p[1];
      ap.z = scanPoses->at(i)->x.p[2];
      pcl_path.push_back(ap);
    }

    for(ScanPose *bl: buf_lba2loop)
    {
      bl->update(dx);
      ap.x = bl->x.p[0];
      ap.y = bl->x.p[1];
      ap.z = bl->x.p[2];
      pcl_path.push_back(ap);
    }
    
    for(int i=0; i<win_count; i++)
    {
      IMUST &x = x_buf[i];
      x.v = dx.R * x.v;
      x.p = dx.R * x.p + dx.p;
      x.R = dx.R * x.R;
      if(g_update == 1)
        x.g = dx.R * x.g;
      // PointType ap;
      ap.x = x.p[0]; ap.y = x.p[1]; ap.z = x.p[2];
      pcl_path.push_back(ap);
    }

    pub_pl_func(pcl_path, pub_curr_path);

    x_curr.R = x_buf[win_count-1].R;
    x_curr.p = x_buf[win_count-1].p;
    x_curr.v = dx.R * x_curr.v;
    x_curr.g = x_buf[win_count-1].g;
    
    for(int i=0; i<win_size; i++)
      mp[i] = i;

    for(ScanPose *bl: buf_lba2loop)
    {
      IMUST xx = bl->x;
      PVec pvec_tem = *(bl->pvec);
      for(pointVar &pv: pvec_tem)
        pv.pnt = xx.R * pv.pnt + xx.p;
      cut_voxel(surf_map, pvec_tem, win_size, 0, &surf_map_lru, surf_map_capacity, &surf_map_slide);
    }
    
    PLV(3) pwld;
    for(int i=0; i<win_count; i++)
    {
      pwld.clear();
      for(pointVar &pv: *pvec_buf[i])
        pwld.push_back(x_buf[i].R * pv.pnt + x_buf[i].p);
      cut_voxel(surf_map, pvec_buf[i], i, surf_map_slide, win_size, pwld, sws[0], &surf_map_lru, surf_map_capacity);
    }

    for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      iter->second->recut(win_count, x_buf, sws[0]);

    if(g_update == 1) g_update = 2;
    loop_detect = 0;
    double t2 = rclcpp::Clock().now().seconds();
    LOG_INFO(PERF, "loop_update rebuild | cost:{:.1f}ms | slide_window:{}", (t2 - t1) * 1000.0, sws[0].size());

    // Before/after chassis-center pose of the current frame across this loop
    // correction, so the drift removed by the loop closure can be compared
    // directly.
    Eigen::Matrix3d R_chassis_after;
    Eigen::Vector3d p_after_loop;
    imu_pose_to_chassis(x_curr.R, x_curr.p, R_chassis_after, p_after_loop);
    Eigen::Quaterniond q_after_loop(R_chassis_after);
    double pos_delta = (p_after_loop - p_before_loop).norm();
    double ang_delta = Log(R_chassis_before.transpose() * R_chassis_after).norm() * 57.3;
    LOG_DECISION(LOOP, "loop closure pose change | before chassis_p:({:.4f} {:.4f} {:.4f}) q:({:.6f} {:.6f} {:.6f} {:.6f}) | after chassis_p:({:.4f} {:.4f} {:.4f}) q:({:.6f} {:.6f} {:.6f} {:.6f}) | delta_p:{:.4f}m delta_ang:{:.4f}deg",
                 p_before_loop[0], p_before_loop[1], p_before_loop[2],
                 q_before_loop.x(), q_before_loop.y(), q_before_loop.z(), q_before_loop.w(),
                 p_after_loop[0], p_after_loop[1], p_after_loop[2],
                 q_after_loop.x(), q_after_loop.y(), q_after_loop.z(), q_after_loop.w(),
                 pos_delta, ang_delta);
  }

  // load the previous keyframe in the local voxel map
  void keyframe_loading(double jour)
  {
    if(history_kfsize <= 0) return;
    double tt1 = rclcpp::Clock().now().seconds();
    PointType ap_curr;
    ap_curr.x = x_curr.p[0];
    ap_curr.y = x_curr.p[1];
    ap_curr.z = x_curr.p[2];
    vector<int> vec_idx;
    vector<float> vec_dis;
    kd_keyframes.radiusSearch(ap_curr, 10, vec_idx, vec_dis);

    for(int id: vec_idx)
    {
      int ord_kf = pl_kdmap->points[id].curvature;
      if(keyframes->at(id)->exist)
      {
        Keyframe &kf = *(keyframes->at(id));
        IMUST &xx = kf.x0;
        PVec pvec; pvec.reserve(kf.plptr->size());

        pointVar pv; pv.var.setZero();
        int plsize = kf.plptr->size();
        // for(int j=0; j<plsize; j+=2)
        for(int j=0; j<plsize; j++)
        {
          PointType ap = kf.plptr->points[j];
          pv.pnt << ap.x, ap.y, ap.z;
          pv.pnt = xx.R * pv.pnt + xx.p;
          pvec.push_back(pv);
        }

        cut_voxel(surf_map, pvec, win_size, jour, &surf_map_lru, surf_map_capacity, &surf_map_slide);
        kf.exist = 0;
        history_kfsize--;
        break;
      }
    }
    
  }

  int initialization(deque<sensor_msgs::msg::Imu::SharedPtr> &imus, Eigen::MatrixXd &hess, LidarFactor &voxhess, PLV(3) &pwld, pcl::PointCloud<PointType>::Ptr pcl_curr)
  {
    static vector<pcl::PointCloud<PointType>::Ptr> pl_origs;
    static vector<double> beg_times;
    static vector<deque<sensor_msgs::msg::Imu::SharedPtr>> vec_imus;

    pcl::PointCloud<PointType>::Ptr orig(new pcl::PointCloud<PointType>(*pcl_curr));
    if(odom_ekf.process(x_curr, *pcl_curr, imus) == 0)
      return 0;

    if(win_count == 0)
      imupre_scale_gravity = odom_ekf.scale_gravity;

    PVecPtr pptr(new PVec);
    double downkd = down_size >= 0.5 ? down_size : 0.5;
    down_sampling_voxel(*pcl_curr, downkd);
    var_init(extrin_para, *pcl_curr, pptr, dept_err, beam_err);
    lio_state_estimation_kdtree(pptr);

    pwld.clear();
    pvec_update(pptr, x_curr, pwld);

    win_count++;
    x_buf.push_back(x_curr);
    pvec_buf.push_back(pptr);
    // Deliberately NOT HighRateOdom::instance().anchor(x_curr) here: until
    // motion_init() succeeds (win_count reaches win_size, checked below),
    // x_curr is only this scan's own lightweight per-scan EKF+kdtree
    // estimate — no sliding-window joint optimization has run on it yet,
    // no cross-scan/gravity/bias consistency constraints — a visibly
    // rougher estimate than what every scan gets once initialization
    // finishes (see the main loop's own anchor() call, after this
    // function stops being called). Anchoring/publishing off of it was
    // producing a real ~1s startup transient (matches win_size=10 scans
    // at ~10Hz) with the same LiDAR-vs-IMU pop this whole class exists to
    // manage, just larger and for a reason unrelated to normal operation.
    // HighRateOdom simply stays un-`ready` (no tf published at all) for
    // this ~1s, instead of publishing something known-rougher.
    ResultOutput::instance().pub_localtraj(pwld, 0, x_curr, sessionNames.size()-1, pcl_path);

    if(win_count > 1)
    {
      imu_pre_buf.push_back(new IMU_PRE(x_buf[win_count-2].bg, x_buf[win_count-2].ba));
      imu_pre_buf[win_count-2]->push_imu(imus);
    }

    pcl::PointCloud<PointType> pl_mid = *orig;
    down_sampling_close(*orig, down_size);
    if(orig->size() < 1000)
    {
      *orig = pl_mid;
      down_sampling_close(*orig, down_size / 2);
    }

    sort(orig->begin(), orig->end(), [](PointType &x, PointType &y)
    {return x.curvature < y.curvature;});

    pl_origs.push_back(orig);
    beg_times.push_back(odom_ekf.pcl_beg_time);
    vec_imus.push_back(imus);

    int is_success = 0;
    if(win_count >= win_size)
    {
      is_success = Initialization::instance().motion_init(pl_origs, vec_imus, beg_times, &hess, voxhess, x_buf, surf_map, surf_map_slide, pvec_buf, win_size, sws, x_curr, imu_pre_buf, extrin_para);

      if(is_success == 0)
        return -1;

      // motion_init() rebuilds surf_map internally via its own cut_voxel()
      // calls that don't know about surf_map_lru (it lives in this class, not
      // Initialization). Rebuild recency from scratch for the map it handed
      // back, treating every voxel as freshly touched.
      surf_map_lru.clear();
      for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
      {
        surf_map_lru.push_front(iter->first);
        iter->second->lru_it = surf_map_lru.begin();
        iter->second->has_lru = true;
      }
      return 1;
    }
    return 0;
  }

  void system_reset(deque<sensor_msgs::msg::Imu::SharedPtr> &imus, const char *reason = "unknown")
  {
    for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
    {
      iter->second->tras_ptr(octos_release);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
    }
    surf_map.clear(); surf_map_slide.clear();
    surf_map_lru.clear();

    x_curr.setZero();
    // Genesis pose, chassis-anchored (identity/no-op without a chassis
    // extrinsic configured) — matches the constructor's initial genesis, so
    // every reset re-lands on the same world frame instead of drifting it.
    x_curr.R = g_R_chassis_imu;
    x_curr.p = g_R_chassis_imu * Eigen::Vector3d(0, 0, 30) + g_t_chassis_imu;
    odom_ekf.mean_acc.setZero();
    odom_ekf.init_num = 0;
    odom_ekf.IMU_init(imus);
    // mean_acc is measured in the IMU's own body axes; rotate it into the
    // world frame via the genesis rotation above before negating it into g
    // (previously relied on x_curr.R(0) always being Identity).
    x_curr.g = -(x_curr.R * odom_ekf.mean_acc) * imupre_scale_gravity;

    for(int i=0; i<imu_pre_buf.size(); i++)
      delete imu_pre_buf[i];
    x_buf.clear(); pvec_buf.clear(); imu_pre_buf.clear();
    pl_tree->clear();

    for(int i=0; i<win_size; i++)
      mp[i] = i;
    win_base = 0; win_count = 0; pcl_path.clear();
    pub_pl_func(pcl_path, pub_cmap);
    HighRateOdom::instance().reset();
    LOG_DECISION(SYS, "system_reset | reason:{}", reason);
  }

  // After local BA, update the map and marginalize the points of oldest scan
  // multi means multiple thread
  void multi_margi(unordered_map<VOXEL_LOC, OctoTree*> &feat_map, double jour, int win_count, vector<IMUST> &xs, LidarFactor &voxopt, vector<SlideWindow*> &sw)
  {
    // for(auto iter=feat_map.begin(); iter!=feat_map.end();)
    // {
    //   iter->second->jour = jour;
    //   iter->second->margi(win_count, 1, xs, voxopt);
    //   if(iter->second->isexist)
    //     iter++;
    //   else
    //   {
    //     iter->second->clear_slwd(sw);
    //     feat_map.erase(iter++);
    //   }
    // }
    // return;

    int thd_num = thread_num;
    vector<vector<OctoTree*>*> octs;
    for(int i=0; i<thd_num; i++) 
      octs.push_back(new vector<OctoTree*>());

    int g_size = feat_map.size();
    if(g_size < thd_num) return;
    vector<thread*> mthreads(thd_num);
    double part = 1.0 * g_size / thd_num;
    int cnt = 0;
    for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
    {
      iter->second->jour = jour;
      octs[cnt]->push_back(iter->second);
      if(octs[cnt]->size() >= part && cnt < thd_num-1)
        cnt++;
    }

    auto margi_func = [](int win_cnt, vector<OctoTree*> *oct, vector<IMUST> xxs, LidarFactor &voxhess)
    {
      for(OctoTree *oc: *oct)
      {
        oc->margi(win_cnt, 1, xxs, voxhess);
      }
    };

    for(int i=1; i<thd_num; i++)
    {
      mthreads[i] = new thread(margi_func, win_count, octs[i], xs, ref(voxopt));
    }
    
    for(int i=0; i<thd_num; i++)
    {
      if(i == 0)
      {
        margi_func(win_count, octs[i], xs, voxopt);
      }
      else
      {
        mthreads[i]->join();
        delete mthreads[i];
      }
    }

    for(auto iter=feat_map.begin(); iter!=feat_map.end();)
    {
      if(iter->second->isexist)
        iter++;
      else
      {
        iter->second->clear_slwd(sw);
        feat_map.erase(iter++);
      }
    }

    for(int i=0; i<thd_num; i++)
      delete octs[i];

  }

  // Determine the plane and recut the voxel map in octo-tree
  void multi_recut(unordered_map<VOXEL_LOC, OctoTree*> &feat_map, int win_count, vector<IMUST> &xs, LidarFactor &voxopt, vector<vector<SlideWindow*>> &sws)
  {
    // for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
    // {
    //   iter->second->recut(win_count, xs, sws[0]);
    //   iter->second->tras_opt(voxopt);
    // }

    int thd_num = thread_num;
    vector<vector<OctoTree*>> octss(thd_num);
    int g_size = feat_map.size();
    if(g_size < thd_num) return;
    vector<thread*> mthreads(thd_num);
    double part = 1.0 * g_size / thd_num;
    int cnt = 0;
    for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
    {
      octss[cnt].push_back(iter->second);
      if(octss[cnt].size() >= part && cnt < thd_num-1)
        cnt++;
    }

    auto recut_func = [](int win_count, vector<OctoTree*> &oct, vector<IMUST> xxs, vector<SlideWindow*> &sw)
    {
      for(OctoTree *oc: oct)
        oc->recut(win_count, xxs, sw);
    };

    for(int i=1; i<thd_num; i++)
    {
      mthreads[i] = new thread(recut_func, win_count, ref(octss[i]), xs, ref(sws[i]));
    }

    for(int i=0; i<thd_num; i++)
    {
      if(i == 0)
      {
        recut_func(win_count, octss[i], xs, sws[i]);
      }
      else
      {
        mthreads[i]->join();
        delete mthreads[i];
      }
    }

    for(int i=1; i<sws.size(); i++)
    {
      sws[0].insert(sws[0].end(), sws[i].begin(), sws[i].end());
      sws[i].clear();
    }

    for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
      iter->second->tras_opt(voxopt);

  }

  // The main thread of odometry and local mapping
  void thd_odometry_localmapping(rclcpp::Node::SharedPtr &n)
  {
    PLV(3) pwld;
    double down_sizes[3] = {0.1, 0.2, 0.4};
    Eigen::Vector3d last_pos(0, 0 ,0);
    double jour = 0;
    int counter = 0;

    pcl::PointCloud<PointType>::Ptr pcl_curr(new pcl::PointCloud<PointType>());
    int motion_init_flag = 1;
    // Set alongside every motion_init_flag=1 (initial + re-init after a
    // reset) and consumed by the first anchor call that follows
    // motion_init() succeeding — see HighRateOdom::anchor_init().
    bool highrate_first_anchor_pending = true;
    pl_tree.reset(new pcl::PointCloud<PointType>());
    vector<pcl::PointCloud<PointType>::Ptr> pl_origs;
    vector<double> beg_times;
    vector<deque<sensor_msgs::msg::Imu::SharedPtr>> vec_imus;
    bool release_flag = false;
    int degrade_cnt = 0;
    // Pose-jump anomaly check state (see the per-scan check below). Reset
    // whenever system_reset() fires so a discontinuity across a reset is
    // never mistaken for a SLAM anomaly.
    bool have_prev_pose_check = false;
    double prev_pose_check_t = 0;
    Eigen::Matrix3d prev_chassis_R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d prev_chassis_p = Eigen::Vector3d::Zero();
    LidarFactor voxhess(win_size);
    const int mgsize = 1;
    Eigen::MatrixXd hess;
    while(rclcpp::ok() && !g_request_shutdown)
    {
      rclcpp::spin_some(n);

      // Periodic self-sample of this thread's own CPU utilization (see
      // sample_thread_cpu()'s comment in voxelslam.hpp) — ~1s cadence, cheap
      // enough to check every iteration since it's dominated by the
      // sleep(0.001) path when there's nothing to process.
      {
        static double last_sample_wall_ms = -1.0;
        double now_ms = rclcpp::Clock().now().seconds() * 1000.0;
        if(last_sample_wall_ms < 0.0 || now_ms - last_sample_wall_ms >= 1000.0)
        {
          sample_thread_cpu(TSLOT_MAIN, now_ms);
          last_sample_wall_ms = now_ms;
        }
      }

      if(loop_detect == 1)
      {
        loop_update(); last_pos = x_curr.p; jour = 0;
        if(registration_pending.exchange(false))
        {
          g_world_registered = true;
          publish_slam_status("TRACKING");
        }
        // Loop closure snaps x_curr retroactively — it's not tied to "this
        // scan's" odometry update (that's what the anchor() calls next to
        // pub_localtraj are for), so without re-anchoring here HighRateOdom
        // would keep dead-reckoning off the pre-loop-closure branch until
        // the next scan happens to come through.
        HighRateOdom::instance().anchor(x_curr);
        // Loop-closure correction can legitimately snap x_curr — not a
        // pose-jump anomaly, so don't compare across it.
        have_prev_pose_check = false;
      }
      
      bool finish_requested = false;
      declare_and_get<bool>(n, "finish", finish_requested, false);
      is_finish = finish_requested;
      if(is_finish)
      {
        publish_slam_status("FINALIZING");
        HighRateOdom::instance().reset();
        break;
      }

      deque<sensor_msgs::msg::Imu::SharedPtr> imus;
      if(!sync_packages(pcl_curr, imus, odom_ekf))
      {
        if(octos_release.size() != 0)
        {
          int msize = octos_release.size();
          if(msize > 1000) msize = 1000;
          for(int i=0; i<msize; i++)
          {
            delete octos_release.back();
            octos_release.pop_back();
          }
          malloc_trim(0);
        }
        else if(release_flag)
        {
          release_flag = false;
          vector<OctoTree*> octos;
          for(auto iter=surf_map.begin(); iter!=surf_map.end();)
          {
            int dis = jour - iter->second->jour;
            if(dis < 700)
            // if(dis < 200)
            {
              iter++;
            }
            else
            {
              octos.push_back(iter->second);
              iter->second->tras_ptr(octos);
              if(iter->second->has_lru)
                surf_map_lru.erase(iter->second->lru_it);
              surf_map.erase(iter++);
            }
          }
          int ocsize = octos.size();
          for(int i=0; i<ocsize; i++)
            delete octos[i];
          octos.clear();
          malloc_trim(0);
        }
        else if(sws[0].size() > 10000)
        {
          for(int i=0; i<500; i++)
          {
            delete sws[0].back();
            sws[0].pop_back();
          }
          malloc_trim(0);
        }

        sleep(0.001);
        continue;
      }

      static int first_flag = 1;
      if (first_flag)
      {
        pcl::PointCloud<PointType> pl;
        pub_pl_func(pl, pub_pmap);
        pub_pl_func(pl, pub_prev_path);
        first_flag = 0;
      }

      double t0 = rclcpp::Clock().now().seconds();
      double t1=0, t2=0, t3=0, t4=0, t5=0, t6=0, t7=0, t8=0;

      if(motion_init_flag)
      {
        int init = initialization(imus, hess, voxhess, pwld, pcl_curr);

        if(init == 1)
        {
          motion_init_flag = 0;
        }
        else
        {
          if(init == -1)
          {
            system_reset(imus, "motion_init_failed");
            have_prev_pose_check = false;
          }
          continue;
        }
      }
      else
      {
        if(odom_ekf.process(x_curr, *pcl_curr, imus) == 0)
          continue;

        pcl::PointCloud<PointType> pl_down = *pcl_curr;
        down_sampling_voxel(pl_down, down_size);

        if(pl_down.size() < 500)
        {
          pl_down = *pcl_curr;
          down_sampling_voxel(pl_down, down_size / 2);
        }

        PVecPtr pptr(new PVec);
        var_init(extrin_para, pl_down, pptr, dept_err, beam_err);

        if(lio_state_estimation(pptr))
        {
          if(degrade_cnt > 0) degrade_cnt--;
        }
        else
          degrade_cnt++;

        pwld.clear();
        pvec_update(pptr, x_curr, pwld);
        if(highrate_first_anchor_pending)
        {
          HighRateOdom::instance().anchor_init(x_curr);
          highrate_first_anchor_pending = false;
          publish_slam_status(g_world_registered ? "TRACKING" : "LOCALIZING");
        }
        else
          HighRateOdom::instance().anchor(x_curr);
        ResultOutput::instance().pub_localtraj(pwld, jour, x_curr, sessionNames.size()-1, pcl_path);

        // Pose-jump anomaly check: two consecutive scans should never move
        // the chassis center by more than its physical motion limits allow
        // (Odometry.pose_jump_max_lin_vel per axis, pose_jump_max_ang_vel
        // for the rotation magnitude) — anything bigger in the time between
        // them means the estimate itself jumped, not the robot. Compared in
        // the chassis frame (not raw IMU/lidar) so a fast in-place rotation
        // doesn't look like a translation jump from the lidar's lever arm.
        {
          Eigen::Matrix3d R_chassis; Eigen::Vector3d p_chassis;
          imu_pose_to_chassis(x_curr.R, x_curr.p, R_chassis, p_chassis);

          if(have_prev_pose_check)
          {
            double dt = x_curr.t - prev_pose_check_t;
            if(dt > 0)
            {
              Eigen::Vector3d dp = (p_chassis - prev_chassis_p).cwiseAbs();
              double max_lin_step = pose_jump_max_lin_vel * dt;
              double ang_step = Log(prev_chassis_R.transpose() * R_chassis).norm();
              double max_ang_step = pose_jump_max_ang_vel * dt;

              if(dp.x() > max_lin_step || dp.y() > max_lin_step || dp.z() > max_lin_step
                 || ang_step > max_ang_step)
              {
                LOG_WARN(EKF, "pose jump anomaly | dt:{:.3f}s dxyz:({:.4f},{:.4f},{:.4f})m "
                              "max_lin_step:{:.4f}m ang:{:.4f}rad max_ang_step:{:.4f}rad — "
                              "possible SLAM fault, t:{:.6f}",
                          dt, dp.x(), dp.y(), dp.z(), max_lin_step, ang_step, max_ang_step, x_curr.t);
              }
            }
            // dt <= 0 (out-of-order/duplicate timestamp): skip silently,
            // nothing meaningful to compare.
          }

          have_prev_pose_check = true;
          prev_pose_check_t = x_curr.t;
          prev_chassis_R = R_chassis;
          prev_chassis_p = p_chassis;
        }

        t1 = rclcpp::Clock().now().seconds();

        win_count++;
        x_buf.push_back(x_curr);
        pvec_buf.push_back(pptr);
        if(win_count > 1)
        {
          imu_pre_buf.push_back(new IMU_PRE(x_buf[win_count-2].bg, x_buf[win_count-2].ba));
          imu_pre_buf[win_count-2]->push_imu(imus);
        }
        
        keyframe_loading(jour);
        voxhess.clear(); voxhess.win_size = win_size;

        // cut_voxel(surf_map, pvec_buf[win_count-1], win_count-1, surf_map_slide, win_size, pwld, sws[0]);
        cut_voxel_multi(surf_map, pvec_buf[win_count-1], win_count-1, surf_map_slide, win_size, pwld, sws, &surf_map_lru, surf_map_capacity);
        t2 = rclcpp::Clock().now().seconds();

        multi_recut(surf_map_slide, win_count, x_buf, voxhess, sws);
        t3 = rclcpp::Clock().now().seconds();


        if(degrade_cnt > degrade_bound)
        {
          LOG_DECISION(EKF, "degrade_cnt exceeded bound | degrade_cnt:{} degrade_bound:{}", degrade_cnt, degrade_bound);
          degrade_cnt = 0;
          system_reset(imus, "degrade_bound_exceeded");
          have_prev_pose_check = false;

          last_pos = x_curr.p; jour = 0;

          mtx_loop.lock();
          buf_lba2loop_tem.swap(buf_lba2loop);
          mtx_loop.unlock();
          reset_flag = 1;

          motion_init_flag = 1;
          highrate_first_anchor_pending = true;
          history_kfsize = 0;

          continue;
        }
      }

      if(win_count >= win_size)
      {
        t4 = rclcpp::Clock().now().seconds();
        
        if(g_update == 2)
        {
          LI_BA_OptimizerGravity opt_lsv;
          vector<double> resis;
          opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, resis, &hess, 5);
          LOG_INFO(EKF, "gravity re-estimated after loop correction | g:({:.4f} {:.4f} {:.4f}) |g|:{:.4f}",
                   x_buf[0].g[0], x_buf[0].g[1], x_buf[0].g[2], x_buf[0].g.norm());
          g_update = 0;
          x_curr.g = x_buf[win_count-1].g;
        }
        else
        {
          LI_BA_Optimizer opt_lsv;
          opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, &hess);
        }

        ScanPose *bl = new ScanPose(x_buf[0], pvec_buf[0]);
        bl->v6 = hess.block<6, 6>(0, DIM).diagonal();
        for(int i=0; i<6; i++) bl->v6[i] = 1.0 / fabs(bl->v6[i]);
        mtx_loop.lock();
        buf_lba2loop.push_back(bl);
        mtx_loop.unlock();

        x_curr.R = x_buf[win_count-1].R;
        x_curr.p = x_buf[win_count-1].p;
        t5 = rclcpp::Clock().now().seconds();

        ResultOutput::instance().pub_localmap(mgsize, sessionNames.size()-1, pvec_buf, x_buf, pcl_path, win_base, win_count);

        multi_margi(surf_map_slide, jour, win_count, x_buf, voxhess, sws[0]);
        t6 = rclcpp::Clock().now().seconds();

        if((win_base + win_count) % 10 == 0)
        {
          double spat = (x_curr.p - last_pos).norm();
          if(spat > 0.5)
          {
            jour += spat;
            last_pos = x_curr.p;
            release_flag = true;
          }
        }

        if(is_save_map)
        {
          for(int i=0; i<mgsize; i++)
          {
            IMUST &xx = x_buf[i];
            FileReaderWriter::instance().append_pose_line(lidar_pose_ofs, xx.t, xx.p, xx.R);
            lidar_pose_ofs.flush();

            if(image_feature_enabled)
            {
              double img_ts;
              vector<uint8_t> img_data;
              if(find_and_consume_nearest_image(xx.t, g_image_time_tol_sec, img_ts, img_data))
              {
                cv::Mat img = cv::imdecode(cv::Mat(img_data), cv::IMREAD_UNCHANGED);
                if(!img.empty())
                {
                  string img_ts_name = format_ts(img_ts);
                  cv::imwrite(savepath + mapname + "/images/" + img_ts_name + ".png", img);
                  FileReaderWriter::instance().append_pose_line(image_pose_ofs, img_ts, xx.p, xx.R);
                  image_pose_ofs.flush();
                }
                else
                  LOG_WARN(CAMERA, "compressed image decode failed | lidar_ts:{:.6f} image_ts:{:.6f}", xx.t, img_ts);
              }
              // No image within tolerance: silently skip this lidar frame's
              // image/pose line — expected under camera dropout/startup lag.
            }
          }
        }

        for(int i=0; i<win_size; i++)
        {
          mp[i] += mgsize;
          if(mp[i] >= win_size) mp[i] -= win_size;
        }

        for(int i=mgsize; i<win_count; i++)
        {
          x_buf[i-mgsize] = x_buf[i];
          PVecPtr pvec_tem = pvec_buf[i-mgsize];
          pvec_buf[i-mgsize] = pvec_buf[i];
          pvec_buf[i] = pvec_tem;
        }

        for(int i=win_count-mgsize; i<win_count; i++)
        {
          x_buf.pop_back();
          pvec_buf.pop_back();

          delete imu_pre_buf.front();
          imu_pre_buf.pop_front();
        }

        win_base += mgsize; win_count -= mgsize;
      }

      // Per-scan cost breakdown, throttled to once every 10 point clouds so
      // it doesn't spam the log at scan rate. Counter is reset (not just
      // incremented forever) so it never overflows across a long-running run.
      static int odom_log_counter = 0;
      bool odom_log_due = t1 > 0 && (++odom_log_counter >= 10);
      if(odom_log_due)
      {
        odom_log_counter = 0;
        double t_end = rclcpp::Clock().now().seconds();
        double mem = get_memory();
        double total_ms = (t_end - t0) * 1000.0;
        double ekf_ms = (t1 - t0) * 1000.0;
        double cutvoxel_ms = (t2 - t1) * 1000.0;
        double recut_ms = (t3 - t2) * 1000.0;
        double ba_ms = (t5 - t4) * 1000.0;
        double margi_ms = (t6 - t5) * 1000.0;
        // Report the chassis-center pose (not the raw IMU pose) here, same
        // convention as pub_localtraj/pub_localmap/save_chassis_traj.
        Eigen::Matrix3d R_chassis_log;
        Eigen::Vector3d p_chassis_log;
        imu_pose_to_chassis(x_curr.R, x_curr.p, R_chassis_log, p_chassis_log);
        Eigen::Quaterniond q_curr(R_chassis_log);
        LOG_INFO(PERF, "odometry loop | total:{:.1f}ms ekf:{:.1f}ms cut_voxel:{:.1f}ms recut:{:.1f}ms ba:{:.1f}ms margi:{:.1f}ms mem:{:.2f}GB jour:{:.1f} | chassis_pose p:({:.4f} {:.4f} {:.4f}) q:({:.6f} {:.6f} {:.6f} {:.6f})",
                 total_ms, ekf_ms, cutvoxel_ms, recut_ms, ba_ms, margi_ms, mem, jour,
                 p_chassis_log[0], p_chassis_log[1], p_chassis_log[2], q_curr.x(), q_curr.y(), q_curr.z(), q_curr.w());
      }
    }

    vector<OctoTree *> octos;
    for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
    {
      iter->second->tras_ptr(octos);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
    }

    for(int i=0; i<octos.size(); i++)
      delete octos[i];
    octos.clear();

    for(int i=0; i<sws[0].size(); i++)
      delete sws[0][i];
    sws[0].clear();
    malloc_trim(0);
  }

  // Build the pose graph in loop closure
  void build_graph(gtsam::Values &initial, gtsam::NonlinearFactorGraph &graph, int cur_id, PGO_Edges &lp_edges, gtsam::noiseModel::Diagonal::shared_ptr default_noise, vector<int> &ids, vector<int> &stepsizes, int lpedge_enable)
  {
    initial.clear(); graph = gtsam::NonlinearFactorGraph();
    ids.clear();
    lp_edges.connect(cur_id, ids);

    stepsizes.clear(); stepsizes.push_back(0);
    for(int i=0; i<ids.size(); i++)
      stepsizes.push_back(stepsizes.back() + multimap_scanPoses[ids[i]]->size());
    
    for(int ii=0; ii<ids.size(); ii++)
    {
      int bsize = stepsizes[ii], id = ids[ii];
      for(int j=bsize; j<stepsizes[ii+1]; j++)
      {
        IMUST &xc = multimap_scanPoses[id]->at(j-bsize)->x;
        gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
        initial.insert(j, pose3);
        if(j > bsize)
        {
          gtsam::Vector samv6(6);
          samv6 = multimap_scanPoses[ids[ii]]->at(j-1-bsize)->v6;
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(samv6);
          add_edge(j-1, j, multimap_scanPoses[id]->at(j-1-bsize)->x, multimap_scanPoses[id]->at(j-bsize)->x, graph, v6_noise);
          // add_edge(j-1, j, multimap_scanPoses[id]->at(j-1-bsize)->x, multimap_scanPoses[id]->at(j-bsize)->x, graph, default_noise);
        }
      }
    }

    if(multimap_scanPoses[ids[0]]->size() != 0)
    {
      int ceil = multimap_scanPoses[ids[0]]->size();
      // if(ceil > 10) ceil = 10;
      ceil = 1;
      for(int i=0; i<ceil; i++)
      {
        Eigen::Matrix<double, 6, 1> v6_fixd;
        v6_fixd << 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9;
        gtsam::noiseModel::Diagonal::shared_ptr fixd_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_fixd));
        IMUST xf = multimap_scanPoses[ids[0]]->at(i)->x;
        gtsam::Pose3 pose3 = gtsam::Pose3(gtsam::Rot3(xf.R), gtsam::Point3(xf.p));
        graph.addPrior(i, pose3, fixd_noise);
      }
    }

    if(lpedge_enable == 1)
    for(PGO_Edge &edge: lp_edges.edges)
    {
      vector<int> step(2);
      if(edge.is_adapt(ids, step))
      {
        int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
        for(int i=0; i<edge.rots.size(); i++)
        {
          int id1 = mp[0] + edge.ids1[i];
          int id2 = mp[1] + edge.ids2[i];
          add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, default_noise);
        }
      }
    }
    
  }

  // The main thread of loop clousre
  // The topDownProcess of HBA is also run here
  void thd_loop_closure(rclcpp::Node::SharedPtr &n)
  {
    pl_kdmap.reset(new pcl::PointCloud<PointType>);
    vector<STDescManager*> std_managers;
    PGO_Edges lp_edges;

    double jud_default = 0.45, icp_eigval = 14;
    double ratio_drift = 0.05;
    int curr_halt = 10, prev_halt = 30;
    int isHighFly = 0;
    // Above this same-session BTC match score (stricter than jud_default, which
    // only gates whether a loop edge is even attempted), a verified match means
    // this candidate is essentially the same place as an already-stored keyframe
    // — the loop edge still gets added for drift correction, but the candidate
    // itself is not kept as a keyframe / added to the BTC descriptor database, so
    // repeatedly revisiting the same spot doesn't keep growing the keyframe pool.
    double redundant_score_thresh = 0.7;
    declare_and_get<double>(n, "Loop.jud_default", jud_default, 0.45);
    declare_and_get<double>(n, "Loop.icp_eigval", icp_eigval, 14);
    declare_and_get<double>(n, "Loop.ratio_drift", ratio_drift, 0.05);
    declare_and_get<int>(n, "Loop.curr_halt", curr_halt, 10);
    declare_and_get<int>(n, "Loop.prev_halt", prev_halt, 30);
    declare_and_get<int>(n, "Loop.isHighFly", isHighFly, 0);
    declare_and_get<double>(n, "Loop.redundant_score_thresh", redundant_score_thresh, 0.7);
    ConfigSetting config_setting;
    read_parameters(n, config_setting, isHighFly);

    vector<double> juds;
    FileReaderWriter::instance().previous_map_names(n, sessionNames, juds);
    FileReaderWriter::instance().pgo_edges_io(lp_edges, sessionNames, 0, savepath, mapname);
    FileReaderWriter::instance().previous_map_read(std_managers, multimap_scanPoses, multimap_keyframes, config_setting, lp_edges, n, sessionNames, juds, savepath, win_size);
    
    STDescManager *std_manager = new STDescManager(config_setting);
    sessionNames.push_back(mapname);
    std_managers.push_back(std_manager);
    multimap_scanPoses.push_back(scanPoses);
    multimap_keyframes.push_back(keyframes);
    juds.push_back(jud_default);
    vector<double> jours(std_managers.size(), 0);

    vector<int> relc_counts(std_managers.size(), prev_halt);
    
    deque<ScanPose*> bl_local;
    Eigen::Matrix<double, 6, 1> v6_init, v6_fixd;
    v6_init << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    v6_fixd << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_init));
    gtsam::noiseModel::Diagonal::shared_ptr fixd_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_fixd));
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;

    vector<int> ids(1, std_managers.size() - 1), stepsizes(2, 0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr plbtc(new pcl::PointCloud<pcl::PointXYZI>);
    IMUST x_key;
    int buf_base = 0;

    while(rclcpp::ok() && !g_request_shutdown)
    {
      // Periodic self-sample of this thread's own CPU utilization — see
      // sample_thread_cpu()'s comment in voxelslam.hpp.
      {
        static double last_sample_wall_ms = -1.0;
        double now_ms = rclcpp::Clock().now().seconds() * 1000.0;
        if(last_sample_wall_ms < 0.0 || now_ms - last_sample_wall_ms >= 1000.0)
        {
          sample_thread_cpu(TSLOT_LOOP, now_ms);
          last_sample_wall_ms = now_ms;
        }
      }

      if(reset_flag == 1)
      {
        reset_flag = 0;
        scanPoses->insert(scanPoses->end(), buf_lba2loop_tem.begin(), buf_lba2loop_tem.end());
        for(ScanPose *bl: buf_lba2loop_tem) bl->pvec = nullptr;
        buf_lba2loop_tem.clear();

        keyframes = new vector<Keyframe*>();
        multimap_keyframes.push_back(keyframes);
        scanPoses = new vector<ScanPose*>();
        multimap_scanPoses.push_back(scanPoses);

        bl_local.clear(); buf_base = 0; 
        std_manager->config_setting_.skip_near_num_ = -(std_manager->plane_cloud_vec_.size()+10);
        std_manager = new STDescManager(config_setting);
        std_managers.push_back(std_manager);
        relc_counts.push_back(prev_halt);
        sessionNames.push_back(mapname + to_string(sessionNames.size()));
        juds.push_back(jud_default);
        jours.push_back(0);

        mapname = sessionNames.back();
        string cmd = "mkdir " + savepath + mapname + "/";
        int ss = system(cmd.c_str());
        if(is_save_map && image_feature_enabled)
          system(("mkdir -p " + savepath + mapname + "/images/").c_str());
        if(is_save_map && !mapname.empty())
          system(("mkdir -p " + savepath + mapname + "/kf/").c_str());
        open_output_streams();

        ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids);
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids, pub_pmap);
        publish_keyframe_pose_array(multimap_keyframes, pub_keyframe_pose_array);

        initial.clear(); graph = gtsam::NonlinearFactorGraph();
        ids.clear(); ids.push_back(std_managers.size()-1);
        stepsizes.clear(); stepsizes.push_back(0); stepsizes.push_back(0);
      }

      if(is_finish && buf_lba2loop.empty())
      {
        break;
      }

      if(buf_lba2loop.empty() || loop_detect == 1)
      {
        sleep(0.01); continue;
      }
      ScanPose *bl_head = nullptr;
      mtx_loop.lock();
      if(!buf_lba2loop.empty()) 
      {
        bl_head = buf_lba2loop.front();
        buf_lba2loop.pop_front();
      }
      mtx_loop.unlock();
      if(bl_head == nullptr) continue;

      int cur_id = std_managers.size() - 1;
      scanPoses->push_back(bl_head);
      bl_local.push_back(bl_head);
      IMUST xc = bl_head->x;
      gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
      int g_pos = stepsizes.back();
      initial.insert(g_pos, pose3);

      if(g_pos > 0)
      {
        gtsam::Vector samv6(scanPoses->at(buf_base-1)->v6);
        gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(samv6);
        add_edge(g_pos-1, g_pos, scanPoses->at(buf_base-1)->x, xc, graph, v6_noise);
      }
      else
      {
        gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
        graph.addPrior(0, pose3, fixd_noise);
      }

      if(buf_base == 0) x_key = xc;
      buf_base++; stepsizes.back() += 1;

      if(bl_local.size() < win_size) continue;
      double ang = Log(x_key.R.transpose() * xc.R).norm() * 57.3;
      double len = (xc.p - x_key.p).norm();
      if(ang < 5 && len < 0.1 && buf_base > win_size)
      {
        bl_local.front()->pvec = nullptr;
        bl_local.pop_front();
        continue;
      }
      for(double &jour: jours)
        jour += len;
      x_key = xc;

      PVecPtr pptr(new PVec);
      for(int i=0; i<win_size; i++)
      {
        ScanPose &bl = *bl_local[i];
        Eigen::Vector3d delta_p = xc.R.transpose() * (bl.x.p - xc.p);
        Eigen::Matrix3d delta_R = xc.R.transpose() *  bl.x.R;
        for(pointVar pv: *(bl.pvec))
        {
          pv.pnt = delta_R * pv.pnt + delta_p;
          pptr->push_back(pv);
        }
      }
      for(int i=0; i<win_size; i++)
      {
        bl_local.front()->pvec = nullptr;
        bl_local.pop_front();
      }

      Keyframe *smp = new Keyframe(xc);
      smp->id = buf_base - 1;
      smp->jour = jours[cur_id];
      down_sampling_pvec(*pptr, voxel_size/10, *(smp->plptr));

      // Use the same bounded, persisted cloud for online and loaded descriptors.
      pcl::copyPointCloud(*smp->plptr, *plbtc);

      vector<STD> stds_vec;
      std_manager->GenerateSTDescs(plbtc, stds_vec, buf_base-1);
      pair<int, double> search_result(-1, 0);
      pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
      vector<pair<STD, STD>> loop_std_pair;

      bool isGraph = false, isOpt = false;
      int match_num = 0;
      // Set below when a same-session match is verified (isPush) with a score
      // high enough to treat this candidate as a duplicate of an existing
      // keyframe — the loop edge still gets added, but this candidate is not
      // kept as a keyframe (see redundant_score_thresh above).
      bool isRedundant = false;

      // Total elapsed time for the whole relocalization pipeline for this
      // keyframe: SearchLoop (all sessions) + icp_normal + (if triggered)
      // build_graph + the ISAM2 joint pose-graph optimization below. The
      // actual LOG_DECISION print is deferred until after that optimization
      // block finishes, so cost reflects everything up to the moment the
      // system is done reacting to the relocalization, not just the ICP step.
      double t_reloc_pipeline0 = rclcpp::Clock().now().seconds();
      bool reloc_first = false, reloc_again = false;
      int reloc_id = -1, reloc_cand_kf = -1, reloc_cur_kf = -1;
      double reloc_icp_rmse = -1.0, reloc_icp_min_eigval = -1.0;
      // Same-session counterpart of the reloc_* bookkeeping above: set when
      // this keyframe's own-session (id == cur_id) branch below pushes a
      // loop edge, so the cost of that path (SearchLoop for id==cur_id +
      // icp_normal + drift check + whatever isGraph/isOpt triggers afterward)
      // can be logged even on iterations that never touch a previous session
      // (see the else-if next to reloc_cost_ms below).
      bool same_session_matched = false;
      int ss_cand_kf = -1, ss_cur_kf = -1;
      double ss_icp_rmse = -1.0, ss_icp_min_eigval = -1.0;

      for(int id=0; id<=cur_id; id++)
      {
        std_managers[id]->SearchLoop(stds_vec, search_result, loop_transform, loop_std_pair, std_manager->plane_cloud_vec_.back());

        if(search_result.first >= 0)
        {
          LOG_INFO(LOOP, "BTC candidate found | session:{} cur_kf:{} cand_kf:{} score:{:.4f}",
                   id, buf_base, search_result.first, search_result.second);
        }

        if(search_result.first >= 0 && search_result.second > juds[id])
        {
          double icp_rmse = -1.0, icp_min_eigval = -1.0;
          bool icp_ok = icp_normal(*(std_manager->plane_cloud_vec_.back()), *(std_managers[id]->plane_cloud_vec_[search_result.first]), loop_transform, icp_eigval, &icp_rmse, &icp_min_eigval);
          if(icp_ok)
          {
            int ord_bl = std_managers[id]->plane_cloud_vec_[search_result.first]->header.seq;

            IMUST &xx = multimap_scanPoses[id]->at(ord_bl)->x;
            double drift_p = (xx.R * loop_transform.first + xx.p - xc.p).norm();

            bool isPush = false;
            int step = -1;
            if(id == cur_id)
            {
              double span = smp->jour - keyframes->at(search_result.first)->jour;
              LOG_INFO(LOOP, "drift check (same-session) | drift:{:.4f} span:{:.4f} ratio:{:.4f} thresh:{:.4f}",
                       drift_p, span, drift_p / span, ratio_drift);

              if(drift_p / span < ratio_drift)
              {
                isPush = true;
                step = stepsizes.size() - 2;

                // Mark this iteration's SearchLoop+icp_normal+drift-check
                // cost (already timed from t_reloc_pipeline0 above) as
                // attributable to the same-session path, for the cost log
                // below — separate from reloc_first/reloc_again, which only
                // fire for a cross-session (id != cur_id) match.
                same_session_matched = true;
                ss_cand_kf = search_result.first; ss_cur_kf = buf_base-1;
                ss_icp_rmse = icp_rmse; ss_icp_min_eigval = icp_min_eigval;

                if(search_result.second >= redundant_score_thresh)
                {
                  isRedundant = true;
                  LOG_DECISION(MAP, "keyframe skipped | reason:redundant_match cur_kf:{} cand_kf:{} score:{:.4f} thresh:{:.4f}",
                               buf_base-1, search_result.first, search_result.second, redundant_score_thresh);
                }

                if(relc_counts[id] > curr_halt && drift_p > 0.10)
                {
                  isOpt = true;
                  for(int &cnt: relc_counts) cnt = 0;
                }
              }
              else
              {
                LOG_DECISION(LOOP, "loop edge rejected by drift (same-session) | drift:{:.4f} span:{:.4f} ratio:{:.4f} thresh:{:.4f}",
                             drift_p, span, drift_p / span, ratio_drift);
              }
            }
            else
            {
              for(int i=0; i<ids.size(); i++)
                if(id == ids[i]) 
                  step = i;
              
              LOG_INFO(LOOP, "drift check (cross-session) | drift:{:.4f} journey:{:.4f} step:{}", drift_p, jours[id], step);

              if(step == -1)
              {
                // First-ever successful match against this previously-loaded
                // session in this run — session `id` is not in `ids`/`stepsizes`
                // yet, so add_edge()/the "loop edge accepted"+RELOCALIZED lines
                // below (gated on step > -1) never fire for this event. The
                // print itself is deferred to after the isOpt block below,
                // since isOpt=true here means the joint pose-graph
                // optimization runs unconditionally on this same iteration
                // (unlike later re-corrections, which are gated on
                // relc_counts/drift_p) and its cost should count toward the
                // total relocalization pipeline time.
                reloc_first = true;
                reloc_id = id; reloc_cand_kf = ord_bl; reloc_cur_kf = buf_base-1;
                reloc_icp_rmse = icp_rmse; reloc_icp_min_eigval = icp_min_eigval;
                isGraph = true;
                isOpt = true;
                relc_counts[id] = 0;
                g_update = 1;
                isPush = true;
                jours[id] = 0;
              }
              else
              {
                if(drift_p / jours[id] < 0.05)
                {
                  jours[id] = 1e-6; // set to 0
                  isPush = true;
                  if(relc_counts[id] > prev_halt && drift_p > 0.25)
                  {
                    isOpt = true;
                    for(int &cnt: relc_counts) cnt = 0;
                  }
                }
                else
                {
                  LOG_DECISION(LOOP, "loop edge rejected by drift (cross-session) | drift:{:.4f} journey:{:.4f} ratio:{:.4f} thresh:0.05",
                               drift_p, jours[id], drift_p / jours[id]);
                }
              }

            }

            if(isPush)
            {
              match_num++;
              lp_edges.push(id, cur_id, ord_bl, buf_base-1, loop_transform.second, loop_transform.first, v6_init);
              if(step > -1)
              {
                int id1 = stepsizes[step] + ord_bl;
                int id2 = stepsizes.back() - 1;
                add_edge(id1, id2, loop_transform.second, loop_transform.first, graph, odom_noise);
                LOG_DECISION(LOOP, "loop edge accepted | (session:{} cur_session:{}) (cand_kf:{} cur_kf:{})", id, cur_id, ord_bl, buf_base-1);
                if(id != cur_id)
                {
                  // rmse/min_eigval are the same fit-quality numbers icp_normal
                  // used internally to accept this match — rmse is the
                  // point-to-plane registration error, min_eigval is how
                  // geometrically distinct the matched normals are (higher =
                  // more confident). The print itself (with total pipeline
                  // cost) is deferred to after the isOpt block below.
                  reloc_again = true;
                  reloc_id = id; reloc_cand_kf = ord_bl; reloc_cur_kf = buf_base-1;
                  reloc_icp_rmse = icp_rmse; reloc_icp_min_eigval = icp_min_eigval;
                }
              }
            }

            // if(isPush)
            // {
            //   icp_check(*(smp->plptr), *(std_managers[id]->plane_cloud_vec_[search_result.first]), pub_test, pub_init, loop_transform, multimap_scanPoses[id]->at(ord_bl)->x);
            // }

          }
        }
        else if(search_result.first >= 0)
        {
          LOG_DECISION(LOOP, "loop candidate rejected by score | session:{} cand_kf:{} score:{:.4f} thresh:{:.4f}",
                       id, search_result.first, search_result.second, juds[id]);
        }

      }
      for(int &it: relc_counts) it++;

      if(isRedundant)
      {
        // Loop edge (if any) was already pushed above for drift correction;
        // just don't grow the keyframe/BTC-descriptor store with a near-duplicate.
        std_manager->plane_cloud_vec_.pop_back();
        delete smp;
      }
      else
      {
        mtx_keyframe.lock();
        keyframes->push_back(smp);
        mtx_keyframe.unlock();
        std_manager->AddSTDescs(stds_vec);

        // Cartographer-submap-style keyframe interface for nav_prob_grid:
        // send this keyframe's cloud+pose exactly once, now that we know
        // it's being kept (not a redundant near-duplicate — a discarded
        // one would otherwise leave a ghost submap downstream that never
        // gets a pose update, since it's not in `keyframes` for
        // publish_keyframe_pose_array to find later).
        if(g_world_registered)
          publish_keyframe_submap(cur_id, *smp, pub_keyframe_submap);

        // Same "kept, not a redundant near-duplicate" timing, archived to
        // disk instead of/as well as published — see save_keyframe_cloud's
        // comment. Only this session's own keyframes are archived here (a
        // relocalized previous session's own keyframes already have their
        // own kf/ from whenever THEY were created, if that session ever ran
        // with is_save_map on).
        if(is_save_map && !mapname.empty())
          FileReaderWriter::instance().save_keyframe_cloud(*smp, savepath, mapname);
      }

      if(isGraph)
      {
        build_graph(initial, graph, cur_id, lp_edges, odom_noise, ids, stepsizes, 1);
      }

      if(isOpt)
      {
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        gtsam::ISAM2 isam(parameters);
        isam.update(graph, initial);

        for(int i=0; i<5; i++) isam.update();
        gtsam::Values results = isam.calculateEstimate();
        int resultsize = results.size();
        
        IMUST x1 = scanPoses->at(buf_base-1)->x;
        int idsize = ids.size();

        history_kfsize = 0;
        for(int ii=0; ii<idsize; ii++)
        {
          int tip = ids[ii];
          for(int j=stepsizes[ii]; j<stepsizes[ii+1]; j++)
          {
            int ord = j - stepsizes[ii];
            multimap_scanPoses[tip]->at(ord)->set_state(results.at(j).cast<gtsam::Pose3>());
          }
        }
        mtx_keyframe.lock();
        for(int ii=0; ii<idsize; ii++)
        {
          int tip = ids[ii];
          for(Keyframe *kf: *multimap_keyframes[tip])
            kf->x0 = multimap_scanPoses[tip]->at(kf->id)->x;
        }
        mtx_keyframe.unlock();

        initial.clear();
        for(int i=0; i<resultsize; i++)
          initial.insert(i, results.at(i).cast<gtsam::Pose3>());
        
        IMUST x3 = scanPoses->at(buf_base-1)->x;
        dx.p = x3.p - x3.R * x1.R.transpose() * x1.p;
        dx.R = x3.R * x1.R.transpose();
        x_key = x3;

        PVec pvec_tem;
        int subsize = keyframes->size();
        int init_num = 5;
        for(int i=subsize-init_num; i<subsize; i++)
        {
          if(i < 0) continue;
          Keyframe &sp = *(keyframes->at(i));
          sp.exist = 0;
          pvec_tem.reserve(sp.plptr->size());
          pointVar pv; pv.var.setZero();
          for(PointType &ap: sp.plptr->points)
          {
            pv.pnt << ap.x, ap.y, ap.z;
            pv.pnt = sp.x0.R * pv.pnt + sp.x0.p;
            for(int j=0; j<3; j++)
              pv.var(j, j) = ap.normal[j];
            pvec_tem.push_back(pv);
          }
          cut_voxel(map_loop, pvec_tem, win_size, 0);
        }

        if(subsize > init_num)
        {
          pl_kdmap->clear();
          for(int i=0; i<subsize-init_num; i++)
          {
            Keyframe &kf = *(keyframes->at(i));
            kf.exist = 1;
            PointType pp;
            pp.x = kf.x0.p[0]; pp.y = kf.x0.p[1]; pp.z = kf.x0.p[2];
            pp.intensity = cur_id; pp.curvature = i;
            pl_kdmap->push_back(pp);
          }

          kd_keyframes.setInputCloud(pl_kdmap);
          history_kfsize = pl_kdmap->size();
        }
        if(reloc_first && !g_world_registered)
        {
          for(Keyframe *kf: *keyframes)
            publish_keyframe_submap(cur_id, *kf, pub_keyframe_submap);
          registration_pending = true;
        }
        loop_detect = 1;

        vector<int> ids2 = ids; ids2.pop_back();
        ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids2);
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_pmap);
        ids2.clear(); ids2.push_back(ids.back());
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_cmap);
        publish_keyframe_pose_array(multimap_keyframes, pub_keyframe_pose_array);

      }



      if(reloc_first || reloc_again)
      {
        // Total elapsed time for the ENTIRE relocalization pipeline for this
        // keyframe: SearchLoop (over every loaded session) + icp_normal +
        // (since isOpt was set) build_graph + the ISAM2 joint optimization
        // above — not just the icp_normal() sub-step.
        double reloc_cost_ms = (rclcpp::Clock().now().seconds() - t_reloc_pipeline0) * 1000.0;
        if(reloc_first)
        {
          LOG_DECISION(LOOP, "FIRST RELOCALIZATION against previous map | session:{} cur_session:{} cand_kf:{} cur_kf:{} cost:{:.1f}ms rmse:{:.4f} min_eigval:{:.4f} — joining session into pose graph, optimized now",
                       reloc_id, cur_id, reloc_cand_kf, reloc_cur_kf, reloc_cost_ms, reloc_icp_rmse, reloc_icp_min_eigval);
        }
        else
        {
          LOG_DECISION(LOOP, "RELOCALIZED against previous map | session:{} cur_session:{} cand_kf:{} cur_kf:{} cost:{:.1f}ms rmse:{:.4f} min_eigval:{:.4f}",
                       reloc_id, cur_id, reloc_cand_kf, reloc_cur_kf, reloc_cost_ms, reloc_icp_rmse, reloc_icp_min_eigval);
        }
      }
      else if(same_session_matched)
      {
        // Same-session counterpart of reloc_cost_ms above, for keyframes
        // that never touch a previous session: total elapsed time for
        // SearchLoop(id==cur_id) + icp_normal + drift check + (if isOpt got
        // set by the same-session branch) build_graph + the ISAM2 joint
        // optimization above. LOG_INFO (not LOG_DECISION) so this stays out
        // of the WARN+ console filter — same-session loop updates are
        // routine, not a "decision" worth surfacing on the terminal.
        double ss_loop_cost_ms = (rclcpp::Clock().now().seconds() - t_reloc_pipeline0) * 1000.0;
        LOG_INFO(LOOP, "same-session loop update | cur_session:{} cand_kf:{} cur_kf:{} cost:{:.1f}ms isOpt:{} rmse:{:.4f} min_eigval:{:.4f}",
                 cur_id, ss_cand_kf, ss_cur_kf, ss_loop_cost_ms, isOpt, ss_icp_rmse, ss_icp_min_eigval);
      }

    }

    for(int i=0; i<std_managers.size(); i++)
      delete std_managers[i];
    malloc_trim(0);

    if(is_finish)
    {
      if(keyframes->empty())
      {
        sessionNames.pop_back();
        std_managers.pop_back();
        multimap_scanPoses.pop_back();
        multimap_keyframes.pop_back();
        juds.pop_back();
        jours.pop_back();
        relc_counts.pop_back();
      }

      if(multimap_keyframes.empty())
      {
        LOG_WARN(SYS, "finish requested but no session has any keyframes | is_save_map:{} — nothing to save/finalize", is_save_map);
        return;
      }

      int cur_id = std_managers.size() - 1;
      build_graph(initial, graph, cur_id, lp_edges, odom_noise, ids, stepsizes, 0);

      topDownProcess(initial, graph, ids, stepsizes);
    }

    if(is_save_map)
    {
      for(int i=0; i<ids.size(); i++)
      {
        FileReaderWriter::instance().save_pose(*(multimap_scanPoses[ids[i]]), sessionNames[ids[i]], "/alidarState.txt", savepath);
        FileReaderWriter::instance().save_chassis_traj(*(multimap_scanPoses[ids[i]]), sessionNames[ids[i]], "/chassis_traj.txt", savepath);
      }

      FileReaderWriter::instance().pgo_edges_io(lp_edges, sessionNames, 1, savepath, mapname);
      if(is_finish && !mapname.empty())
        publish_keyframe_pose_array(multimap_keyframes, pub_keyframe_pose_array, true,
                                    savepath + mapname + "/", mapname);
    }

    for(int i=0; i<multimap_scanPoses.size(); i++)
    {
      for(int j=0; j<multimap_scanPoses[i]->size(); j++)
        delete multimap_scanPoses[i]->at(j);
    }
    for(int i=0; i<multimap_keyframes.size(); i++)
    {
      for(int j=0; j<multimap_keyframes[i]->size(); j++)
        delete multimap_keyframes[i]->at(j);
    }
    
    malloc_trim(0);
  }

  // The top down process of HBA
  void topDownProcess(gtsam::Values &initial, gtsam::NonlinearFactorGraph &graph, vector<int> &ids, vector<int> &stepsizes)
  {
    cnct_map = ids;
    gba_size = multimap_keyframes.back()->size();
    gba_flag = 1;

    pcl::PointCloud<PointType> pl0;
    pub_pl_func(pl0, pub_pmap);
    pub_pl_func(pl0, pub_cmap);
    pub_pl_func(pl0, pub_curr_path);
    pub_pl_func(pl0, pub_prev_path);
    pub_pl_func(pl0, pub_scan);

    while(gba_flag && rclcpp::ok() && !g_request_shutdown)
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if(g_request_shutdown) return;
    
    for(PGO_Edge &edge: gba_edges1.edges)
    {
      vector<int> step(2);
      if(edge.is_adapt(ids, step))
      {
        int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
        for(int i=0; i<edge.rots.size(); i++)
        {
          int id1 = mp[0] + edge.ids1[i];
          int id2 = mp[1] + edge.ids2[i];
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(edge.covs[i]));
          add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, v6_noise);
        }
      }
    }

    for(PGO_Edge &edge: gba_edges2.edges)
    {
      vector<int> step(2);
      if(edge.is_adapt(ids, step))
      {
        int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
        for(int i=0; i<edge.rots.size(); i++)
        {
          int id1 = mp[0] + edge.ids1[i];
          int id2 = mp[1] + edge.ids2[i];
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(edge.covs[i]));
          add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, v6_noise);
        }
      }
    }

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters);
    isam.update(graph, initial);

    for(int i=0; i<5; i++) isam.update();
    gtsam::Values results = isam.calculateEstimate();
    int resultsize = results.size();

    int idsize = ids.size();
    for(int ii=0; ii<idsize; ii++)
    {
      int tip = ids[ii];
      for(int j=stepsizes[ii]; j<stepsizes[ii+1]; j++)
      {
        int ord = j - stepsizes[ii];
        multimap_scanPoses[tip]->at(ord)->set_state(results.at(j).cast<gtsam::Pose3>());
      }
    }

    Eigen::Quaterniond qq(multimap_scanPoses[0]->at(0)->x.R);

    for(int ii=0; ii<idsize; ii++)
    {
      int tip = ids[ii];
      for(Keyframe *smp: *multimap_keyframes[tip])
        smp->x0 = multimap_scanPoses[tip]->at(smp->id)->x;
    }

    ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids);
    vector<int> ids2 = ids; ids2.pop_back();
    ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_pmap);
    ids2.clear(); ids2.push_back(ids.back());
    ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_cmap);
    publish_keyframe_pose_array(multimap_keyframes, pub_keyframe_pose_array);
  }

  // The bottom up to add edge in HBA
  void HBA_add_edge(vector<IMUST> &p_xs, vector<Keyframe*> &p_smps, PGO_Edges &gba_edges, vector<int> &maps, int max_iter, int thread_num, pcl::PointCloud<PointType>::Ptr plptr = nullptr)
  {
    if(p_smps.size() < 2)
    {
      if(plptr && !p_smps.empty()) *plptr = *p_smps.front()->plptr;
      return; // No inter-keyframe constraint exists in a stationary one-keyframe map.
    }
    bool is_display = false;
    if(plptr == nullptr) is_display = true;

    vector<Keyframe*> smps;
    vector<IMUST> xs;
    int last_mp = -1, isCnct = 0;
    for(int i=0; i<p_smps.size(); i++)
    {
      Keyframe *smp = p_smps[i];
      if(smp->mp != last_mp)
      {
        isCnct = 0;
        for(int &m: maps)
        if(smp->mp == m)
        {
          isCnct = 1; break;
        }
        last_mp = smp->mp;
      }

      if(isCnct)
      {
        smps.push_back(smp);
        xs.push_back(p_xs[i]);
      }
    }
    
    int wdsize = smps.size();
    Eigen::MatrixXd hess;
    vector<double> gba_eigen_value_array_orig = gba_eigen_value_array;
    double gba_min_eigen_value_orig = gba_min_eigen_value;
    double gba_voxel_size_orig = gba_voxel_size;

    int up = 4;
    int converge_flag = 0;
    double converge_thre = 0.05;

    for(int iterCnt = 0; iterCnt < max_iter; iterCnt++)
    {
      if(converge_flag == 1 || iterCnt == max_iter-1)
      {
        // if(plptr == nullptr)
        // {
        //   break;
        // }

        gba_voxel_size = voxel_size;
        gba_eigen_value_array = plane_eigen_value_thre;
        gba_min_eigen_value = min_eigen_value;
      }

      unordered_map<VOXEL_LOC, OctreeGBA*> oct_map;
      for(int i=0; i<wdsize; i++)
        OctreeGBA::cut_voxel(oct_map, xs[i], smps[i]->plptr, i, wdsize);

      LidarFactor voxhess(wdsize);
      OctreeGBA_multi_recut(oct_map, voxhess, thread_num);

      Lidar_BA_Optimizer opt_lsv;
      opt_lsv.thd_num = thread_num;
      vector<double> resis;
      bool is_converge = opt_lsv.damping_iter(xs, voxhess, &hess, resis, up, is_display);
      if(is_display)
        LOG_DEBUG(BACKEND, "GBA submap iter | resi_delta_ratio:{:.6f}", fabs(resis[0] - resis[1]) / resis[0]);
      if((fabs(resis[0] - resis[1]) / resis[0] < converge_thre && is_converge) || (iterCnt == max_iter-2 && converge_flag == 0))
      {
        converge_thre = 0.01;
        if(converge_flag == 0)
        {
          converge_flag = 1;
        }
        else if(converge_flag == 1)
        {
          break;
        }
      }
    }

    gba_eigen_value_array = gba_eigen_value_array_orig;
    gba_min_eigen_value = gba_min_eigen_value_orig;
    gba_voxel_size = gba_voxel_size_orig;

    for(int i=0; i<wdsize - 1; i++)
    for(int j=i+1; j<wdsize; j++)
    {
      bool isAdd = true;
      Eigen::Matrix<double, 6, 1> v6;
      for(int k=0; k<6; k++)
      {
        double hc = fabs(hess(6*i+k, 6*j+k));
        if(hc < 1e-6) // 1e-6
        {
          isAdd = false; break;
        }
        v6[k] = 1.0 / hc;
      }

      if(isAdd)
      {
        Keyframe &s1 = *smps[i]; Keyframe &s2 = *smps[j];
        Eigen::Vector3d tra = xs[i].R.transpose() * (xs[j].p - xs[i].p);
        Eigen::Matrix3d rot = xs[i].R.transpose() *  xs[j].R;
        gba_edges.push(s1.mp, s2.mp, s1.id, s2.id, rot, tra, v6);
      }
    }

    if(plptr != nullptr)
    {
      pcl::PointCloud<PointType> pl;
      IMUST xc = xs[0];
      for(int i=0; i<wdsize; i++)
      {
        Eigen::Vector3d dp = xc.R.transpose() * (xs[i].p - xc.p);
        Eigen::Matrix3d dR = xc.R.transpose() *  xs[i].R;
        for(PointType ap: smps[i]->plptr->points)
        {
          Eigen::Vector3d v3(ap.x, ap.y, ap.z);
          v3 = dR * v3 + dp;
          ap.x = v3[0]; ap.y = v3[1]; ap.z = v3[2];
          ap.intensity = smps[i]->mp;
          pl.push_back(ap);
        }
      }
      
      down_sampling_voxel(pl, voxel_size / 8);
      plptr->clear(); plptr->reserve(pl.size());
      for(PointType &ap: pl.points)
        plptr->push_back(ap);
    }
    else
    {
      // pcl::PointCloud<PointType> pl, path;
      // pub_pl_func(pl, pub_test);
      // for(int i=0; i<wdsize; i++)
      // {
      //   PointType pt;
      //   pt.x = xs[i].p[0]; pt.y = xs[i].p[1]; pt.z = xs[i].p[2];
      //   path.push_back(pt);
      //   for(int j=1; j<smps[i]->plptr->size(); j+=2)
      //   {
      //     PointType ap = smps[i]->plptr->points[j];
      //     Eigen::Vector3d v3(ap.x, ap.y, ap.z);
      //     v3 = xs[i].R * v3 + xs[i].p;
      //     ap.x = v3[0]; ap.y = v3[1]; ap.z = v3[2];
      //     ap.intensity = smps[i]->mp;
      //     pl.push_back(ap);

      //     if(pl.size() > 1e7)
      //     {
      //       pub_pl_func(pl, pub_test);
      //       pl.clear();
      //       sleep(0.05);
      //     }
      //   }
      // }
      // pub_pl_func(pl, pub_test);
      // return;
    }
  }

  // The main thread of bottom up in global mapping
  void thd_globalmapping(rclcpp::Node::SharedPtr &n)
  {
    declare_and_get<double>(n, "GBA.voxel_size", gba_voxel_size, 1.0);
    declare_and_get<double>(n, "GBA.min_eigen_value", gba_min_eigen_value, 0.01);
    declare_and_get<vector<double>>(n, "GBA.eigen_value_array", gba_eigen_value_array, vector<double>());
    for(double &iter: gba_eigen_value_array) iter = 1.0 / iter;
    int total_max_iter = 1;
    declare_and_get<int>(n, "GBA.total_max_iter", total_max_iter, 1);

    vector<Keyframe*> gba_submaps;
    deque<int> localID;

    int smp_mp = 0;
    int buf_base = 0;
    int wdsize = 10;
    int mgsize = 5;
    int thread_num = 5;

    while(rclcpp::ok() && !g_request_shutdown)
    {
      // Periodic self-sample of this thread's own CPU utilization — see
      // sample_thread_cpu()'s comment in voxelslam.hpp.
      {
        static double last_sample_wall_ms = -1.0;
        double now_ms = rclcpp::Clock().now().seconds() * 1000.0;
        if(last_sample_wall_ms < 0.0 || now_ms - last_sample_wall_ms >= 1000.0)
        {
          sample_thread_cpu(TSLOT_GBA, now_ms);
          last_sample_wall_ms = now_ms;
        }
      }

      if(multimap_keyframes.empty())
      {
        sleep(0.1); continue;
      }

      int smp_flag = 0;
      if(smp_mp+1 < multimap_keyframes.size() && !multimap_keyframes.back()->empty())
        smp_flag = 1;

      vector<Keyframe*> &smps = *multimap_keyframes[smp_mp];
      int total_ba = 0;
      if(gba_flag == 1 && smp_mp >= cnct_map.back() && gba_size <= buf_base)
      {
        LOG_INFO(BACKEND, "GBA triggered | smp_mp:{} gba_size:{} buf_base:{}", smp_mp, gba_size, buf_base);
        total_ba = 1;
      }
      else if(smps.size() <= buf_base)
      {
        if(smp_flag == 0)
        {
          sleep(0.1); continue;
        }
      }
      else
      {
        smps[buf_base]->mp = smp_mp;
        localID.push_back(buf_base);

        buf_base++;
        if(localID.size() < wdsize)
        {
          sleep(0.1); continue;
        }
      }

      vector<IMUST> xs;
      vector<Keyframe*> smp_local;
      mtx_keyframe.lock();
      for(int i: localID)
      {
        xs.push_back(multimap_keyframes[smp_mp]->at(i)->x0);
        smp_local.push_back(multimap_keyframes[smp_mp]->at(i));
      }
      mtx_keyframe.unlock();

      Keyframe *gba_smp = new Keyframe(smp_local[0]->x0);
      vector<int> mps{smp_mp};
      HBA_add_edge(xs, smp_local, gba_edges1, mps, 1, 2, gba_smp->plptr);
      gba_smp->id = smp_local[0]->id;
      gba_smp->mp = smp_mp;
      gba_submaps.push_back(gba_smp);

      if(total_ba == 1)
      {
        LOG_INFO(BACKEND, "GBA final pass | gba_size:{} gba_submaps:{}", gba_size, gba_submaps.size());
        vector<IMUST> xs;
        mtx_keyframe.lock();
        for(Keyframe *smp: gba_submaps)
        {
          xs.push_back(multimap_scanPoses[smp->mp]->at(smp->id)->x);
        }
        mtx_keyframe.unlock();
        gba_edges2.edges.clear(); gba_edges2.mates.clear();
        HBA_add_edge(xs, gba_submaps, gba_edges2, cnct_map, total_max_iter, thread_num);

        if(is_finish)
        {
          for(int i=0; i<gba_submaps.size(); i++)
            delete gba_submaps[i];
        }
        gba_submaps.clear();

        malloc_trim(0);
        gba_flag = 0;
      }
      else if(smp_flag == 1 && multimap_keyframes[smp_mp]->size() <= buf_base)
      {
        smp_mp++; buf_base = 0; localID.clear();
        // printf("switch: %d\n", smp_mp);
      }
      else
      {
        for(int i=0; i<mgsize; i++)
          localID.pop_front();
      }
  
    }

  }

};

int main(int argc, char **argv)
{
  // Disable rclcpp's built-in SIGINT handler: by default it shuts the DDS
  // context down directly from the signal handler, which races with
  // whichever worker thread below happens to be mid-spin/mid-publish at
  // that instant and segfaults on exit. Install our own handler that just
  // flips a flag; the while(rclcpp::ok() && !g_request_shutdown) loops
  // notice it, the threads return, main() joins them, and only then is
  // rclcpp::shutdown() called.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, voxelslam_sigint_handler);
  std::signal(SIGTERM, voxelslam_sigint_handler);

  rclcpp::Node::SharedPtr n = std::make_shared<rclcpp::Node>("cmn_voxel");
  vxlm_log::init(n->get_logger());
  HighRateOdom::instance().open_log();
  g_node = n;
  pub_slam_status = n->create_publisher<std_msgs::msg::String>(
    "/slam/status", rclcpp::QoS(1).reliable().transient_local());
  string previous_map;
  declare_and_get<string>(n, "General.previous_map", previous_map, "");
  g_world_registered = previous_map.empty();
  publish_slam_status("INITIALIZING");
  // See g_imu_cbg's comment (voxelslam.hpp): false so this group is NOT
  // auto-added to whatever executor spins the node (spin_some(n) in
  // thd_odometry_localmapping) — it's only serviced by the dedicated
  // executor/thread set up below, after VOXEL_SLAM's constructor has
  // created sub_imu into it.
  g_imu_cbg = n->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  pub_cmap = n->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cmap", 100);
  pub_pmap = n->create_publisher<sensor_msgs::msg::PointCloud2>("/map_pmap", 100);
  pub_scan = n->create_publisher<sensor_msgs::msg::PointCloud2>("/map_scan", 100);
  pub_scan_filtered = n->create_publisher<sensor_msgs::msg::PointCloud2>("/map_scan_filtered", 100);
  pub_keyframe_submap = n->create_publisher<astribot_slam_msgs::msg::KeyframeSubmap>("/voxel_slam/keyframe_submap", 100);
  pub_keyframe_pose_array = n->create_publisher<astribot_slam_msgs::msg::KeyframePoseArray>("/voxel_slam/keyframe_pose_array", 10);
  pub_init = n->create_publisher<sensor_msgs::msg::PointCloud2>("/map_init", 100);
  pub_test = n->create_publisher<sensor_msgs::msg::PointCloud2>("/map_test", 100);
  pub_curr_path = n->create_publisher<sensor_msgs::msg::PointCloud2>("/map_path", 100);
  pub_prev_path = n->create_publisher<sensor_msgs::msg::PointCloud2>("/map_true", 100);
  // 20Hz aft_mapped tf tick, upsampled from the ~10Hz LiDAR-corrected pose via
  // IMU propagation — see highrate_odom.hpp. General.enable_highrate_odom
  // (default on) can fall this back to the old ~10Hz LiDAR-frame-rate
  // publish without reverting code, for A/B testing against motion control.
  int enable_highrate_odom = 1;
  declare_and_get<int>(n, "General.enable_highrate_odom", enable_highrate_odom, 1);
  HighRateOdom::instance().set_enabled(enable_highrate_odom != 0);
  double max_pose_age = 0.5;
  declare_and_get<double>(n, "General.max_pose_age_sec", max_pose_age, 0.5);
  if(!std::isfinite(max_pose_age) || max_pose_age <= 0)
    throw std::invalid_argument("General.max_pose_age_sec must be positive");
  HighRateOdom::instance().set_max_pose_age(max_pose_age);
  hr_odom_timer = n->create_wall_timer(std::chrono::milliseconds(50),
    [](){ HighRateOdom::instance().publish_tick(); }, g_imu_cbg);

  robot_self_filter.configure(n);
  feat.robot_mask = [](const Eigen::Vector3d &point) { return robot_self_filter.contains(point); };
  VOXEL_SLAM vs(n);
  mp = new int[vs.win_size];
  for(int i=0; i<vs.win_size; i++)
    mp[i] = i;

  // Dedicated executor/thread for g_imu_cbg (sub_imu + hr_odom_timer) — see
  // its comment in voxelslam.hpp. spin() blocks efficiently (no busy-wait)
  // until either callback is ready, so this thread stays responsive to IMU
  // messages and the 20Hz tick regardless of how long
  // thd_odometry_localmapping spends inside a scan's optimization.
  rclcpp::executors::SingleThreadedExecutor imu_executor;
  imu_executor.add_callback_group(g_imu_cbg, n->get_node_base_interface());
  thread thread_imu([&imu_executor](){ imu_executor.spin(); });

  thread thread_loop(&VOXEL_SLAM::thd_loop_closure, &vs, ref(n));
  thread thread_gba(&VOXEL_SLAM::thd_globalmapping, &vs, ref(n));
  vs.thd_odometry_localmapping(n);

  thread_loop.join();
  pub_keyframe_pose_array->wait_for_all_acked(std::chrono::seconds(2));
  g_request_shutdown = true;
  thread_gba.join();
  if(vs.is_finish) publish_slam_status("FINISHED");
  // imu_executor.spin() only returns once cancelled (or the context shuts
  // down) — it won't notice g_request_shutdown on its own like the
  // thread_loop/thread_gba while-loops do.
  imu_executor.cancel();
  thread_imu.join();
  rclcpp::shutdown();

  // Deliberately skip normal process exit here. pub_cmap/pub_scan/.../g_node
  // (declared as file-scope globals in voxelslam.hpp) and the function-local
  // `static tf2_ros::TransformBroadcaster br` inside HighRateOdom::publish_tf
  // (highrate_odom.hpp) only get destroyed by the C++ runtime *after* main()
  // returns, i.e. after
  // rclcpp::shutdown() above has already torn down the DDS context. Their
  // destructors then try to clean up rcl publisher/subscription/tf handles
  // against an already-dead context, which is exactly the
  // "cannot publish data" / "Fail in delete datareader" segfault seen on
  // Ctrl-C. All real work (map saving, thread cleanup) is already done by
  // this point, so skip static/global destruction entirely and let the OS
  // reclaim resources on process exit instead of racing rmw's teardown.
  std::_Exit(0);
}
