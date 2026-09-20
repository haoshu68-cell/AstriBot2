#include <set>
#include <string>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/gui/Plugin.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>
#include <tinyxml2.h>

namespace astribot
{
// This plugin runs only in the GUI process. Server lidar rendering and physics
// retain the proxy geometry; making the SDF visual invisible would affect both.
class SocialProxyDisplay : public ignition::gui::Plugin
{
  Q_OBJECT

protected:
  void LoadConfig(const tinyxml2::XMLElement *config) override
  {
    title = "Social proxy display";
    for (auto elem = config->FirstChildElement("proxy_model"); elem;
         elem = elem->NextSiblingElement("proxy_model")) {
      if (elem->GetText()) models_.emplace_back(elem->GetText());
    }
    if (auto window = ignition::gui::App()->findChild<ignition::gui::MainWindow *>())
      window->installEventFilter(this);
  }

  bool eventFilter(QObject *object, QEvent *event) override
  {
    if (event->type() == ignition::gui::events::Render::kType) {
      for (const auto &name : ignition::rendering::loadedEngines()) {
        auto engine = ignition::rendering::engine(name);
        if (!engine) continue;
        for (unsigned int s = 0; s < engine->SceneCount(); ++s) {
          auto scene = engine->SceneByIndex(s);
          if (!scene) continue;
          for (unsigned int v = 0; v < scene->VisualCount(); ++v) {
            auto visual = scene->VisualByIndex(v);
            if (!visual || !IsProxy(visual->Name())) continue;
            visual->SetVisible(false);
            if (reported_.insert(visual->Name()).second)
              ignmsg << "[SocialProxyDisplay] GUI-only hide: " << visual->Name() << std::endl;
          }
        }
      }
    }
    return QObject::eventFilter(object, event);
  }

private:
  bool IsProxy(const std::string &name) const
  {
    for (const auto &model : models_) {
      const auto pos = name.find(model);
      if (pos == std::string::npos) continue;
      const auto end = pos + model.size();
      if ((pos == 0 || (pos >= 2 && name.compare(pos - 2, 2, "::") == 0)) &&
          (end == name.size() || name.compare(end, 2, "::") == 0)) return true;
    }
    return false;
  }

  std::vector<std::string> models_;
  std::set<std::string> reported_;
};
}

IGNITION_ADD_PLUGIN(astribot::SocialProxyDisplay, ignition::gui::Plugin)
#include "social_proxy_display.moc"
