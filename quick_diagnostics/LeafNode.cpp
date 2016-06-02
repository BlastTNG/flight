#include "LeafNode.h"

LeafNode::LeafNode(GetData::Dirfile* dirfile, const char* fieldCode) : StatusNode(fieldCode), dirfile(dirfile), fieldCode(fieldCode) {}

// Emit a clicked signal when this label is pressed
void LeafNode::mousePressEvent(QMouseEvent* evt) {
  emit clicked(fieldCode);
}
