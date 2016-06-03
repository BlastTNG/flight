#include "LeafNode.h"

LeafNode::LeafNode(GetData::Dirfile* dirfile, const char* fieldCode) : StatusNode(fieldCode), dirfile(dirfile), fieldCode(fieldCode) {

 }

// TODO: read the most recent sample from dirfile, update color accordingly
void LeafNode::updateStatus() {
  float r = rand() / (float) RAND_MAX;
  if (0.9 < r) {
    setStatus(BAD);
  } else {
    setStatus(GOOD);
  }
  updateStyle();
}

// Emit a clicked signal when this label is pressed
// NB: evt is not used
void LeafNode::mousePressEvent(QMouseEvent* /*evt*/) {
  emit clicked(fieldCode);
}
