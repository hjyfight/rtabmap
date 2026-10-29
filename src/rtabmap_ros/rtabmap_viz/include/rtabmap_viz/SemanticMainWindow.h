#ifndef RTABMAP_VIZ_SEMANTIC_MAIN_WINDOW_H
#define RTABMAP_VIZ_SEMANTIC_MAIN_WINDOW_H

#include <rtabmap/gui/MainWindow.h>
#include <rtabmap_semantic_msgs/msg/semantic_map.hpp>
#include <set>

class QAction;

namespace rtabmap_viz {

// Keeps annotations outside RTAB-Map's node labels and graph constraints.
class SemanticMainWindow : public rtabmap::MainWindow
{
  Q_OBJECT
public:
  SemanticMainWindow(rtabmap::PreferencesDialog * preferences,
                     const std::string & mapKey, double textScale, double labelOffset);
  void setSemanticMap(const rtabmap_semantic_msgs::msg::SemanticMap & map);
  size_t semanticLabelCount() const { return rendered_.size(); }

private Q_SLOTS:
  void redrawSemantics();

private:
  void clearSemantics();
  std::string mapKey_;
  double textScale_;
  double labelOffset_;
  QAction * showSemantics_;
  rtabmap_semantic_msgs::msg::SemanticMap semantics_;
  std::set<std::string> rendered_;
  std::map<std::string, std::pair<Eigen::Vector3d, std::string> > drawingCache_;
};
}
#endif
