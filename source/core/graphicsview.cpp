#include "graphicsview.h"

#include <QResizeEvent>

GraphicsView::GraphicsView(QWidget* parent) :
    QGraphicsView (parent)
{
    setFocusPolicy(Qt::StrongFocus);
    setAcceptDrops (true);
}

void GraphicsView::resizeEvent(QResizeEvent *event) {
    QGraphicsView::resizeEvent(event);
    if (scene())
    {
        scene()->setSceneRect(QRect(QPoint(0, 0), event->size()));
        emit sceneRectChanged (QRectF (0, 0, event->size().width(), event->size().height()));
    }

}
