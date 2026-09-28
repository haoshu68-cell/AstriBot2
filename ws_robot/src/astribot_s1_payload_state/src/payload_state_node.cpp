// Evidence reconciliation only: no attach/detach, joint, gripper or navigation commands.
#include "astribot_s1_payload_state/ledger.hpp"
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <random>
#include <thread>
namespace astribot::payload {
namespace {
int64_t steady() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
std::string boot() {std::random_device entropy;std::string seed;for(int i=0;i<8;++i)seed+=std::to_string(entropy());return digest(seed+std::to_string(steady()));}
}
class PayloadStateNode:public rclcpp::Node {
  using Service=moveit_msgs::srv::GetPlanningScene;
  struct Queued {Observation::ConstSharedPtr value;Receipt receipt;};
  std::mutex mutex_;std::condition_variable wake_;std::deque<Queued> queue_;
  bool overflow_=false;std::atomic<bool> stop_{false};std::thread worker_;
  std::unique_ptr<Journal> journal_;std::unique_ptr<Ledger> ledger_;
  rclcpp::Publisher<State>::SharedPtr publisher_;
  rclcpp::Subscription<Observation>::SharedPtr observation_;
  rclcpp::Client<Service>::SharedPtr scene_;
  template<class T> T setting(const std::string &name,const T &value) {
    rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;return declare_parameter<T>(name,value,d);
  }
  void run() {
    std::optional<rclcpp::Client<Service>::FutureAndRequestId> future;
    std::optional<Ticket> ticket;int64_t last_request=-1,last_publish=-1;
    try {
      while(!stop_) {
        std::deque<Queued> batch;bool overflow=false;
        {std::unique_lock<std::mutex> lock(mutex_);wake_.wait_for(lock,std::chrono::milliseconds(10),[this]{return stop_ || !queue_.empty() || overflow_;});
          batch.swap(queue_);overflow=std::exchange(overflow_,false);}
        if(stop_)break;
        auto now=get_clock()->now().nanoseconds(),wall=steady();
        if(overflow) {ledger_->source_fault("ATTACHMENT_INPUT_QUEUE_OVERFLOW",now,wall);batch.clear();}
        for(const auto &event:batch)ledger_->observe(*event.value,get_clock()->now().nanoseconds(),steady(),event.receipt);
        now=get_clock()->now().nanoseconds();wall=steady();
        if(future && future->wait_for(std::chrono::seconds(0))==std::future_status::ready) {
          try {
            auto response=future->get();const bool reconciled=ledger_->reconcile(*ticket,response->scene,now,wall);
            RCLCPP_INFO(get_logger(),"PAYLOAD_SCENE_TIMING event=ready request=%lld generation=%llu sent_ros=%lld sent_steady=%lld handled_ros=%lld handled_steady=%lld elapsed_ns=%lld reconciled=%d",
              static_cast<long long>(future->request_id),static_cast<unsigned long long>(ticket->generation),
              static_cast<long long>(ticket->ros_at),static_cast<long long>(ticket->steady_at),
              static_cast<long long>(now),static_cast<long long>(wall),static_cast<long long>(wall-ticket->steady_at),reconciled);
          }
          catch(const std::exception &e) {RCLCPP_WARN(get_logger(),"Scene readback failed: %s; PAYLOAD_SCENE_TIMING event=error request=%lld generation=%llu sent_ros=%lld sent_steady=%lld handled_ros=%lld handled_steady=%lld elapsed_ns=%lld",
            e.what(),static_cast<long long>(future->request_id),static_cast<unsigned long long>(ticket->generation),
            static_cast<long long>(ticket->ros_at),static_cast<long long>(ticket->steady_at),
            static_cast<long long>(now),static_cast<long long>(wall),static_cast<long long>(wall-ticket->steady_at));}
          future.reset();ticket.reset();
        }
        if(future && wall-ticket->steady_at>=kLease) {
          RCLCPP_WARN(get_logger(),"PAYLOAD_SCENE_TIMING event=timeout request=%lld generation=%llu sent_ros=%lld sent_steady=%lld handled_ros=%lld handled_steady=%lld elapsed_ns=%lld",
            static_cast<long long>(future->request_id),static_cast<unsigned long long>(ticket->generation),
            static_cast<long long>(ticket->ros_at),static_cast<long long>(ticket->steady_at),
            static_cast<long long>(now),static_cast<long long>(wall),static_cast<long long>(wall-ticket->steady_at));
          scene_->remove_pending_request(future->request_id);future.reset();ticket.reset();
        }
        if(!future && (last_request<0 || wall-last_request>=100000000) && scene_->service_is_ready()) {
          ticket=ledger_->request(now,wall);
          if(ticket) {
            auto request=std::make_shared<Service::Request>();
            request->components.components=moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
            future=scene_->async_send_request(request);last_request=wall;
            RCLCPP_INFO(get_logger(),"PAYLOAD_SCENE_TIMING event=send request=%lld generation=%llu sent_ros=%lld sent_steady=%lld submitted_steady=%lld",
              static_cast<long long>(future->request_id),static_cast<unsigned long long>(ticket->generation),
              static_cast<long long>(ticket->ros_at),static_cast<long long>(ticket->steady_at),static_cast<long long>(steady()));
          }
        }
        if(last_publish<0 || wall-last_publish>=50000000 || overflow || !batch.empty()) {
          publisher_->publish(ledger_->state(now,wall));last_publish=wall;
        }
      }
    }catch(const std::exception &e) {
      RCLCPP_ERROR(get_logger(),"Payload evidence worker stopped: %s",e.what());
      ledger_->source_fault("ATTACHMENT_WORKER_FAILED",get_clock()->now().nanoseconds(),steady());
      publisher_->publish(ledger_->state(get_clock()->now().nanoseconds(),steady()));
    }
    if(future)scene_->remove_pending_request(future->request_id);
  }
public:
  PayloadStateNode():Node("payload_state") {
    Config config;config.environment=setting<std::string>("environment","simulation");config.session=setting<std::string>("session_id","");
    config.source=setting<std::string>("source_id","");config.ledger_epoch=boot();
    const auto links=setting<std::vector<std::string>>("allowed_attachment_links",{"astribot_arm_left_tcp_link","astribot_arm_right_tcp_link"});
    config.allowed_links={links.begin(),links.end()};
    // Required session/source and journal path: no auto-empty fallback or persisted authorization.
    journal_=std::make_unique<Journal>(setting<std::string>("journal_path",""));
    ledger_=std::make_unique<Ledger>(config,[this](const auto &record){journal_->append(record);});
    publisher_=create_publisher<State>("/payload/attachment_state",rclcpp::QoS(4).reliable());
    scene_=create_client<Service>("/get_planning_scene");
    observation_=create_subscription<Observation>("/payload/attachment_observation",rclcpp::QoS(32).reliable(),
      [this](Observation::ConstSharedPtr value) {
        const Receipt received{get_clock()->now().nanoseconds(),steady()};
        // Oversize messages are discarded before queueing. DDS wire-size limits are a deployment concern.
        std::lock_guard<std::mutex> lock(mutex_);
        if(queue_.size()>=32 || value->objects.size()>32) {queue_.clear();overflow_=true;}
        else queue_.push_back({std::move(value),received});
        wake_.notify_one();
      });
    worker_=std::thread([this]{run();});
  }
  ~PayloadStateNode() override {stop_=true;wake_.notify_all();if(worker_.joinable())worker_.join();}
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<astribot::payload::PayloadStateNode>());}
  catch(const std::exception &e) {std::cerr<<"payload_state: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
