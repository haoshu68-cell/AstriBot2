#define main reviewed_viewer_main
#include "replay_viewer.cpp"
#undef main
#include <QCoreApplication>
#include <QJsonDocument>
#include <QJsonObject>
namespace {Json results;QWidget*window=nullptr;QPlainTextEdit*detail=nullptr;QListWidget*list=nullptr;QDoubleSpinBox*seconds=nullptr;
void finish(){auto path=std::getenv("REVIEW_UI_REPORT");if(path)std::ofstream(path)<<results.dump(2)<<'\n';QApplication::quit();}
void schedule(){QTimer::singleShot(800,[]{
 for(auto w:QApplication::topLevelWidgets())if(w->windowTitle().startsWith("OFFLINE"))window=w;
 if(!window){results["error"]="no window";finish();return;}detail=window->findChild<QPlainTextEdit*>();list=window->findChild<QListWidget*>();for(auto s:window->findChildren<QDoubleSpinBox*>())if(s->suffix()==" s")seconds=s;
 results["before_event_click"]=detail->toPlainText().toStdString();list->setCurrentRow(0);results["selected_event_detail"]=detail->toPlainText().toStdString();for(auto l:window->findChildren<QLabel*>())if(l->text().contains("时间"))results["label_after_event_click"]=l->text().toStdString();
 if(const char*p=std::getenv("REVIEW_UI_IMAGE"))window->grab().save(p);
 seconds->setValue(0.);for(auto b:window->findChildren<QPushButton*>())if(b->text()=="跳转并重建")b->click();
 QTimer::singleShot(850,[]{results["detail_after_seek_before_first_event"]=detail->toPlainText().toStdString();for(auto l:window->findChildren<QLabel*>())if(l->text().contains("时间"))results["label_after_seek"]=l->text().toStdString();finish();});
 });}
}
Q_COREAPP_STARTUP_FUNCTION(schedule)
int main(int argc,char**argv){return reviewed_viewer_main(argc,argv);}
