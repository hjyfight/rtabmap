#include "rtabmap_viz/SemanticMainWindow.h"
#include <rtabmap/gui/CloudViewer.h>
#include <rtabmap/gui/PreferencesDialog.h>
#include <rtabmap/core/RtabmapEvent.h>
#include <rtabmap/core/Parameters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkActor.h>
#include <QApplication>
#include <QElapsedTimer>
#include <QThread>
#include <QTemporaryDir>
#include <QFile>
#include <QTimer>
#include <QMessageBox>
#include <iostream>
#include <stdexcept>

class InspectableCloud : public rtabmap::CloudViewer
{
public:
  double textX(const std::string & id) {
    auto actors = visualizer()->getShapeActorMap();
    // PCL adds one '*' per viewport index; semantic text uses foreground layer 3.
    auto found = actors->find(id + "***");
    if (found == actors->end()) throw std::runtime_error("Missing text actor: " + id);
    auto actor = vtkActor::SafeDownCast(found->second);
    if (!actor) throw std::runtime_error("Text is not a VTK actor");
    return actor->GetPosition()[0];
  }
};

class TemporaryPreferences : public rtabmap::PreferencesDialog
{
public:
  explicit TemporaryPreferences(const QString & directory) : directory_(directory) {
    rtabmap::Parameters::writeINI(getIniFilePath().toStdString(), {
      {rtabmap::Parameters::kRtabmapWorkingDirectory(), directory_.toStdString()}});
  }
  QString getIniFilePath() const override { return directory_+"/gui.ini"; }
  QString getTmpIniFilePath() const override { return directory_+"/gui.tmp.ini"; }
  QString getDefaultWorkingDirectory() const override { return directory_; }
private:
  QString directory_;
};

class InspectableWindow : public rtabmap_viz::SemanticMainWindow
{
public:
  InspectableWindow(rtabmap::PreferencesDialog * prefs) : SemanticMainWindow(prefs, "/tmp/gui_check.db", 0.12, 0.2) {
    view = new InspectableCloud;
    setCloudViewer(view);
  }
  void graph(double x) {
    std::map<int, rtabmap::Transform> poses{{1, rtabmap::Transform(x, 0, 0, 0, 0, 0)}};
    processRtabmapEvent3DMap(rtabmap::RtabmapEvent3DMap({}, poses, {}));
  }
  InspectableCloud * view;
};

void spinFor(int milliseconds)
{
  QElapsedTimer elapsed; elapsed.start();
  while (elapsed.elapsed() < milliseconds) {
    QApplication::processEvents();
    QThread::msleep(10);
  }
}

void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char ** argv)
{
  std::cerr << "Creating QApplication" << std::endl;
  QApplication application(argc, argv);
  std::cerr << "Creating temporary preferences" << std::endl;
  QTimer::singleShot(8000, []() {
    for (auto widget : QApplication::topLevelWidgets()) {
      if (auto message = qobject_cast<QMessageBox *>(widget)) {
        std::cerr << "Unexpected test dialog: " << message->text().toStdString() << std::endl;
        std::abort();
      }
    }
  });
  QTemporaryDir directory;
  auto preferences = new TemporaryPreferences(directory.path());
  std::cerr << "Creating native map window" << std::endl;
  InspectableWindow window(preferences);
  std::cout << "Native window initialized" << std::endl;
  window.setWindowTitle("RTAB-Map[*] semantic integration check — synthetic map");
  window.resize(1200, 800);
  window.show();
  try {
    window.graph(0);
    rtabmap_semantic_msgs::msg::SemanticMap map;
    map.map_key = "/tmp/gui_check.db";
    map.map_uuid = "gui-test";
    rtabmap_semantic_msgs::msg::SemanticObject object;
    object.object_id = 1; object.label = "chair"; object.confidence = 0.92;
    rtabmap_semantic_msgs::msg::Observation observation;
    observation.node_id = 1; observation.node_stamp = 1; observation.weight = 1;
    observation.position.x = 0.25; observation.position.z = 0.5;
    object.observations.push_back(observation);
    map.objects.push_back(object);
    window.setSemanticMap(map);
    spinFor(300);
    require(window.semanticLabelCount() == 1, "Label did not appear");
    const auto id = std::string("semantic_object_1");
    require(std::abs(window.view->textX(id) - 0.25) < 1e-5, "Wrong initial label position");
    window.view->removeAllTexts();
    spinFor(300);
    require(window.view->getAddedTexts().count(id), "Label lost after original map cleared texts");
    window.graph(1);
    spinFor(300);
    require(std::abs(window.view->textX(id) - 1.25) < 1e-5, "Label did not follow graph correction");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int x=0; x<35; ++x) for (int y=0; y<35; ++y) {
      pcl::PointXYZRGB point;
      point.x = 0.4f+x*0.04f; point.y = -0.7f+y*0.04f; point.z = 0;
      point.r = 100; point.g = 125; point.b = 150; cloud->push_back(point);
    }
    window.view->addCloud("synthetic_floor", cloud);
    window.view->setCloudPointSize("synthetic_floor", 3);
    window.view->setCameraPosition(3, -3, 2.5, 1.25, 0, 0.4, 0, 0, 1);
    window.view->refreshView();
    spinFor(500);
    if (argc > 1) require(window.grab().save(QString::fromLocal8Bit(argv[1])), "Screenshot save failed");
    map.objects.clear();
    window.setSemanticMap(map);
    spinFor(100);
    require(window.semanticLabelCount() == 0 && !window.view->getAddedTexts().count(id), "Deleted label remained");
    std::cout << "{\"passed\":true,\"native_text_actor\":true,\"redraw\":true,\"loop_shift_m\":1.0,\"delete\":true}" << std::endl;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
  return 0;
}
