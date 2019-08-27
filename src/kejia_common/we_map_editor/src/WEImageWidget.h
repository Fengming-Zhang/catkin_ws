#ifndef WEIMAGEWIDGET_H
#define WEIMAGEWIDGET_H

#include <QPixmap>
#include <QWidget>
#include <QString>
#include <QLabel>
#include <QVector>
#include "gs_mapb.h"
#include "topo_map.h"

enum WorkType{ROOM,DOOR,PASSWAY,FURNITURE,ADD_LINE,ERASER,RESET,PrintScreen};
struct RoomRect
{
    QString roomName;
    QRect   roomRect;
};

class WEImageWidget : public QWidget
{
    Q_OBJECT
public:
    WEImageWidget(gslam::GridMapBase* map_, QWidget * parent = 0);

    bool LoadImageFile(const QImage & file);
    bool UpdateImageFile(const QImage & file);
    QSize minimumSize() const;
    QSize sizeHint() const;



signals:
    void selectedRect(QRect &rect, WorkType &type);
    void selectedLine(QLine &line, WorkType &type,QVector<RoomRect> &rooms);
    void showTex(QString tex);

public slots:
    void addRect(QRect &rect,QString &type,QString &name);
    void addLine(QLine &line,QString &type,QString &name);
    void setWorkType(WorkType &type);
    void loadNewTopoFile(std::vector<Entity> m_entities);


protected:
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void wheelEvent(QWheelEvent *event);

private:

    void updateImage();
    void updateImageSmart();
    QPoint client2showImg(QPoint point);
    QPoint showImg2rawImg(QPoint point);
    QPoint client2rawImg(QPoint point);
    QPoint showImg2client(QPoint point);
    QPoint rawImg2showImg(QPoint point);
    QPoint rawImg2client(QPoint point);
    void updateZoom(float val);
    void drawRects(QPainter & painter);
    void drawLines(QPainter & painter);

    WorkType workType;
    QRect rubberBandRect;
    bool rubberBandIsShown;
    QLine doorLine;
    bool doorLineIsShown;

    float zoom;
    QSize clientSize;
    QPoint centerPoint;
    QPoint paintOffset;
    QPixmap pixmap;
    QPixmap showmap;

    QPoint dragStart;
    QPoint dragEnd;

    QVector<QRect> rects;

    QVector<QLine> lines;

    QVector<RoomRect>rooms;
    QVector<QRect>notRooms;
    gslam::GridMapBase* map;

};

#endif // WEIMAGEWIDGET_H
