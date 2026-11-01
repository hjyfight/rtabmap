#include "rtabmap_viz/SemanticMainWindow.h"
#include "rtabmap_viz/SemanticLabels.h"
#include <rtabmap/gui/CloudViewer.h>
#include <rtabmap/gui/PreferencesDialog.h>
#include <rtabmap/core/RtabmapEvent.h>
#include <rtabmap/core/Parameters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkPropCollection.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkTextProperty.h>
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
  vtkRenderer * textRenderer() {
    auto renderers = visualizer()->getRendererCollection();
    renderers->InitTraversal();
    while (auto renderer = renderers->GetNextItem()) {
      if (renderer->GetLayer() == 3) return renderer;
    }
    throw std::runtime_error("Missing text renderer");
  }
  vtkBillboardTextActor3D * textActor() {
    auto props = textRenderer()->GetViewProps();
    props->InitTraversal();
    while (auto prop = props->GetNextProp()) {
      if (auto actor = vtkBillboardTextActor3D::SafeDownCast(prop)) return actor;
    }
    return nullptr;
  }
  double textX() {
    auto actor = textActor();
    if (!actor) throw std::runtime_error("Missing Chinese text actor");
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
    require(rtabmap_viz::semanticDisplayLabel("chair", 1, .92) == "椅子", "Default label includes extra fields");
    require(rtabmap_viz::semanticDisplayLabel("chair", 1, .92, true) == "椅子 #1 0.92", "Detailed formatting was lost");
    require(rtabmap_viz::semanticClassNameChinese("dining table") == "餐桌", "Multiword class was not translated");
    require(rtabmap_viz::semanticClassNameChinese("自定义类别") == "自定义类别", "Chinese custom label was lost");
    require(rtabmap_viz::semanticClassNameChinese("custom_class") == "物体", "Missing Chinese fallback");
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
    require(std::abs(window.view->textX() - 0.25) < 1e-5, "Wrong initial label position");
    require(std::string(window.view->textActor()->GetInput()) == "椅子", "Visible label is not Chinese class only");
    require(QFile::exists(window.view->textActor()->GetTextProperty()->GetFontFile()), "Chinese font missing");
    map.objects[0].confidence = 0.23;
    window.setSemanticMap(map);
    require(std::string(window.view->textActor()->GetInput()) == "椅子", "Confidence change leaked into visible text");
    require(map.objects[0].label == "chair" && map.objects[0].object_id == 1, "Original metadata changed");
    auto second = object;
    second.object_id = 2;
    second.observations[0].position.x = 0.8;
    map.objects.push_back(second);
    window.setSemanticMap(map);
    require(window.semanticLabelCount() == 2, "Same-class objects were merged by their translated name");
    map.objects.pop_back();
    window.setSemanticMap(map);
    window.view->removeAllTexts();
    window.view->textRenderer()->RemoveViewProp(window.view->textActor());
    spinFor(300);
    require(window.view->textActor() != nullptr, "Label lost after map cleared its text actor");
    window.graph(1);
    spinFor(300);
    require(std::abs(window.view->textX() - 1.25) < 1e-5, "Label did not follow graph correction");
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
    require(window.semanticLabelCount() == 0 && !window.view->textActor(), "Deleted label remained");
    require(!window.view->getAddedSpheres().count(id+"_point"), "Deleted object's marker remained");
    std::cout << "{\"passed\":true,\"chinese_class_only\":true,\"metadata_preserved\":true,\"native_text_actor\":true,\"redraw\":true,\"loop_shift_m\":1.0,\"delete\":true}" << std::endl;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
  return 0;
}
