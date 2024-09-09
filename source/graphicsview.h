#ifndef GRAPHICSVIEW_H
#define GRAPHICSVIEW_H

#include <QGraphicsView>

class GraphicsView : public QGraphicsView
{
    Q_OBJECT

public:
    explicit GraphicsView(QWidget* parent = nullptr);

signals:
    void sceneRectChanged(const QRectF &rect);

protected:
    virtual void resizeEvent(QResizeEvent *event);
};

#endif // GRAPHICSVIEW_H
