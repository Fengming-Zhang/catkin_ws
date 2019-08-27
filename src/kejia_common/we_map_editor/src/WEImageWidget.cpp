#include <QtGui>
#include "WEImageWidget.h"
#include "gs_mapb.h"

WEImageWidget::WEImageWidget(gslam::GridMapBase *map_, QWidget *parent)
    :QWidget(parent)
{
    setBackgroundRole(QPalette::WindowText);
    setAutoFillBackground(true);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setFocusPolicy(Qt::StrongFocus);
    rubberBandIsShown = false;
    doorLineIsShown = false;
    workType = ROOM;
    zoom = 1.0;
    map = map_;

    adjustSize();


}

bool WEImageWidget::LoadImageFile(const QImage &file)
{
    pixmap.convertFromImage(file);
    showmap = pixmap.copy();
    zoom = 1.0;
    rubberBandIsShown = false;
    centerPoint.setX(pixmap.width()/2);
    centerPoint.setY(pixmap.height()/2);
    updateImageSmart();
    return true;
}

bool WEImageWidget::UpdateImageFile(const QImage &file)
{
    pixmap.convertFromImage(file);
    showmap = pixmap.copy();
    updateImageSmart();
    return true;
}

QSize WEImageWidget::minimumSize() const
{
    return QSize(1920, 1200);
}

QSize WEImageWidget::sizeHint() const
{
    return QSize(1920, 1200);
}



void WEImageWidget::paintEvent(QPaintEvent *event)
{
    QStylePainter painter(this);

    painter.drawPixmap(paintOffset.rx(), paintOffset.ry(), showmap);

    drawRects(painter);
    drawLines(painter);
    if (rubberBandIsShown)
    {
        painter.setPen(Qt::red);
        painter.drawRect(rubberBandRect.normalized().adjusted(0, 0, -1, -1));
    }

    if (doorLineIsShown)
    {
        painter.setPen(Qt::red);
        painter.drawLine(doorLine);

    }

    return;
}

void WEImageWidget::resizeEvent(QResizeEvent *event)
{
    clientSize = size();
    updateImageSmart();
    return;
}

void WEImageWidget::mousePressEvent(QMouseEvent *event)
{
    if(pixmap.isNull())
        return;

    if(event->button() == Qt::LeftButton)
    {
        if(workType == ROOM || workType == FURNITURE || workType == ERASER||workType==PrintScreen||workType == RESET)
        {
            rubberBandIsShown = true;
            rubberBandRect.setTopLeft(event->pos());
            rubberBandRect.setBottomRight(event->pos());
        }
        else if(workType == DOOR || workType == PASSWAY || workType == ADD_LINE)
        {
            doorLineIsShown = true;
            doorLine.setP1(event->pos());
            doorLine.setP2(event->pos());
        }

        setCursor(Qt::CrossCursor);
    }
    else if(event->button() == Qt::RightButton)
    {
        dragStart = dragEnd = event->pos();
    }
    QPoint mousePose = client2rawImg(event->pos());
    gslam::Point p = map->map2world(mousePose.x(), mousePose.y());
    emit showTex(tr(" Pix x:%1 y:%2  World Pose:[%3, %4]").arg(mousePose.x()).arg(mousePose.y()).arg(QString::number(p.x, 'f', 3)).arg(QString::number(p.y, 'f', 3)));
    update();
    return;
}

void WEImageWidget::mouseMoveEvent(QMouseEvent *event)
{
    if(pixmap.isNull())
        return;

    QPoint mousePose = client2rawImg(event->pos());
    gslam::Point p = map->map2world(mousePose.x(), mousePose.y());
    emit showTex(tr(" Pix x:%1 y:%2  World Pose:[%3, %4]").arg(mousePose.x()).arg(mousePose.y()).arg(QString::number(p.x, 'f', 3)).arg(QString::number(p.y, 'f', 3)));


    if(event->buttons() == Qt::RightButton)
    {
        dragEnd = event->pos();
        centerPoint -= (dragEnd - dragStart) / zoom;
        updateImageSmart();
        dragStart = dragEnd = event->pos();
    }
    else if(event->buttons() == Qt::LeftButton)
    {
        if (rubberBandIsShown)
        {
            rubberBandRect.setBottomRight(event->pos());
        }
        else if(doorLineIsShown)
        {
            doorLine.setP2(event->pos());
        }
    }
    update();
    return;
}

void WEImageWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if(pixmap.isNull())
        return;
    if(event->button() == Qt::LeftButton)
    {
        if(rubberBandIsShown)
        {
            rubberBandIsShown = false;
            rubberBandRect.setBottomRight(event->pos());
            if(rubberBandRect.topLeft() != rubberBandRect.bottomRight())
            {
                if(workType==PrintScreen)
                {

                    QPoint A=client2rawImg(rubberBandRect.topLeft());
                    QPoint B=client2rawImg(rubberBandRect.bottomRight());

                    QDir *temp = new QDir;
                    bool exist = temp->exists("./pdf");
                    if(exist)
                        QMessageBox::warning(this,tr("Create New Document"),tr("Docement is exist!"));
                    else
                    {
                        bool ok = temp->mkdir("./pdf");
                        if( ok )
                            QMessageBox::warning(this,tr("Create New Document"),tr("Create Successfully!"));
                    }

                    QFile myfile("pdf/pdf.txt");
                    myfile.open(QIODevice::WriteOnly);


                    QString sss("offsetx:%1 offsety:%2");
                    QString text=sss.arg(A.x()).arg(A.y()).arg(B.x()).arg(B.y());

                    char*  ch;

                    QByteArray ba = text.toLatin1();

                    ch=ba.data();

                    myfile.write(ch);

                    myfile.close();

                    QPixmap pix=QPixmap::grabWidget(this,rubberBandRect.topLeft().x(),rubberBandRect.topLeft().y(),rubberBandRect.bottomRight().x()-rubberBandRect.topLeft().x(),rubberBandRect.bottomRight().y()-rubberBandRect.topLeft().y());
                    if(pix.isNull())
                    {
                       QMessageBox::information(this,"error","grab Screen failed",QMessageBox::Ok);
                    }
                    else
                    {
                        if(pix.save( "pdf/grab.jpg", "JPG" )==false)
                            QMessageBox::information(this,"right","save error",QMessageBox::Ok);

                        else
                           QMessageBox::information(this,"Grab","bitmap saved as pdf/grab.jpg",QMessageBox::Ok);
                   }




                }
                else
                {
                    QRect select(client2rawImg(rubberBandRect.topLeft()), client2rawImg(rubberBandRect.bottomRight()));
                    select = select.normalized();
                    emit selectedRect(select, workType);
                    emit showTex(tr(" Select rect [%1 %2] - [%3 %4]").arg(select.topLeft().x()).arg(select.topLeft().y()).arg(select.bottomRight().x()).arg(select.bottomRight().y()));
                }
            }
        }
        else if(doorLineIsShown)
        {
            doorLineIsShown = false;
            doorLine.setP2(event->pos());
            if(doorLine.p1() != doorLine.p2())
            {
                QLine tempLine(client2rawImg(doorLine.p1()), client2rawImg(doorLine.p2()));
                emit selectedLine(tempLine, workType,rooms);
                emit showTex(tr(" Select door [%1 %2] - [%3 %4]").arg(tempLine.x1()).arg(tempLine.y1()).arg(tempLine.x2()).arg(tempLine.y2()));
            }
        }
        unsetCursor();
    }

    return;
}

void WEImageWidget::keyPressEvent(QKeyEvent *event)
{
    if(pixmap.isNull())
        return;
    int scale = 1;
    if(event->modifiers() == Qt::ShiftModifier)
    {
        scale = 2;
    }
    switch(event->key())
    {
    case Qt::Key_Plus:
        updateZoom(0.02 * scale);
        break;

    case Qt::Key_Minus:
        updateZoom(-0.02 * scale);
        break;

    case Qt::Key_Up:
        centerPoint.setY(centerPoint.y() + 2 * scale);
        break;

    case Qt::Key_Down:
        centerPoint.setY(centerPoint.y() - 2 * scale);
        break;

    case Qt::Key_Left:
        centerPoint.setX(centerPoint.x() + 2 * scale);
        break;

    case Qt::Key_Right:
        centerPoint.setX(centerPoint.x() - 2 * scale);
        break;

    default:
        QWidget::keyPressEvent(event);
        break;
    }
    updateImageSmart();
    return;
}

void WEImageWidget::wheelEvent(QWheelEvent *event)
{
    if(pixmap.isNull())
        return;
    int num = event->delta()/8;
    num = num / 15;
    updateZoom(num * 0.02);
    updateImageSmart();
    return;
}

void WEImageWidget::updateImage()
{
    showmap = pixmap.scaled(pixmap.size() * zoom,Qt::KeepAspectRatio);
    int xoff = clientSize.width()/2 - centerPoint.x() * zoom;
    int yoff = clientSize.height()/2 - centerPoint.y() * zoom;
    paintOffset.setX(xoff);
    paintOffset.setY(yoff);
    update();
}

void WEImageWidget::updateImageSmart()
{
    if(pixmap.isNull())
        return;
    QPoint showltinclient = showImg2client(QPoint(0,0));
    showltinclient.setX(showltinclient.x() < 0 ? 0 : showltinclient.x());
    showltinclient.setY(showltinclient.y() < 0 ? 0 : showltinclient.y());

    QPoint clientltinraw = client2rawImg(QPoint(0,0));
    clientltinraw.setX(clientltinraw.x() > 0 ? clientltinraw.x() : 0);
    clientltinraw.setY(clientltinraw.y() > 0 ? clientltinraw.y() : 0);

    QPoint clientrdinraw = client2rawImg(QPoint(clientSize.width(),clientSize.height()));
    clientrdinraw.setX(clientrdinraw.x() > pixmap.width() ? pixmap.width() : clientrdinraw.x());
    clientrdinraw.setY(clientrdinraw.y() > pixmap.height() ? pixmap.height() : clientrdinraw.y());

    QSize rawImgSize(clientrdinraw.x() - clientltinraw.x(), clientrdinraw.y() - clientltinraw.y());
    QSize newImgSize = rawImgSize * zoom;

    showmap = pixmap.copy(QRect(clientltinraw,rawImgSize));
    showmap = showmap.scaled(newImgSize);
    paintOffset = showltinclient;
    update();
    return;

}

QPoint WEImageWidget::client2showImg(QPoint point)
{
    return point -= QPoint(clientSize.width()/2 - centerPoint.x() * zoom,clientSize.height()/2 - centerPoint.y() * zoom);

}

QPoint WEImageWidget::showImg2rawImg(QPoint point)
{
    point -= QPoint(centerPoint.x() * zoom, centerPoint.y() * zoom);
    point /= zoom;
    return point += centerPoint;
}
QPoint WEImageWidget::client2rawImg(QPoint point)
{
    return showImg2rawImg(client2showImg(point));
}

QPoint WEImageWidget::showImg2client(QPoint point)
{
    return point += QPoint(clientSize.width()/2 - centerPoint.x() * zoom, clientSize.height()/2 - centerPoint.y() * zoom);
}

QPoint WEImageWidget::rawImg2showImg(QPoint point)
{
    point -= centerPoint;
    point *= zoom;
    return point += QPoint(centerPoint.x() * zoom, centerPoint.y() * zoom);
}

QPoint WEImageWidget::rawImg2client(QPoint point)
{
    return showImg2client(rawImg2showImg(point));
}

void WEImageWidget::updateZoom(float val)
{
    zoom += val;
    zoom = zoom > 10 ? 10 : (zoom < 0.01 ? 0.01 : zoom);
    emit showTex(tr(" Zoom: %1").arg(zoom));
}

void WEImageWidget::addRect(QRect &rect,QString &type,QString &name)
{
    rects.push_back(rect);

    if(type==QString("room"))
    {
      RoomRect temp;
      temp.roomName=name;
      temp.roomRect=rect;
      rooms.push_back(temp);

    }
    else
      notRooms.push_back(rect);
    update();

}

void WEImageWidget::addLine(QLine &line,QString &type,QString &name)
{
    lines.push_back(line);
    update();
}

void WEImageWidget::drawRects(QPainter &painter)
{
    if (workType!=PrintScreen)
    {
        painter.setPen(Qt::blue);
        for(int i = 0; i < rects.size(); i++)
        {
            painter.drawRect(QRect(rawImg2client(rects[i].topLeft()), rawImg2client(rects[i].bottomRight())));

        }
        for(int i=0;i<rooms.size();i++)
        {
            painter.drawText(QRect(rawImg2client(rooms[i].roomRect.topLeft()), rawImg2client(rooms[i].roomRect.bottomRight())).bottomLeft(),rooms[i].roomName);


        }
    }
    else
    {

        for(int i=0;i<notRooms.size();i++)
        {
            painter.fillRect(QRect(rawImg2client(notRooms[i].topLeft()), rawImg2client(notRooms[i].bottomRight())),Qt::gray);

        }



    }

}

void WEImageWidget::drawLines(QPainter &painter)
{
    if(workType!=PrintScreen)
    {
        painter.setPen(Qt::red);

        QLine tempLine;
        double angle;
        for(int i = 0; i < lines.size(); i++)
        {
            tempLine = QLine(rawImg2client(lines[i].p1()), rawImg2client(lines[i].p2()));
            painter.drawLine(tempLine);

            angle = (180.0 / 3.1415) * atan2(tempLine.y2() - tempLine.y1(), tempLine.x2() - tempLine.x1());
            painter.translate(tempLine.p2().x(), tempLine.p2().y());
            painter.rotate(angle);

            painter.drawText(0,0,">>");

            painter.rotate(-angle);
            painter.translate(-tempLine.p2().x(), -tempLine.p2().y());
        }
    }
}

 void WEImageWidget::loadNewTopoFile(std::vector<Entity>  m_entities)
 {
     lines.clear();
     rects.clear();
     rooms.clear();
     notRooms.clear();

     for(int i=0;i<m_entities.size();i++)
     {
         Entity temp=m_entities[i];

         qDebug("load: %s %s %d %d %d %d",temp.class_id.c_str(),temp.entity_name.c_str(),temp.x1,temp.y1,temp.x2,temp.y2);

         if(temp.class_id=="door"||temp.class_id=="passway")
            lines.push_back(QLine(temp.x1,temp.y1,temp.x2,temp.y2));
         if(temp.class_id=="room"||temp.class_id=="furniture")
         {
             rects.push_back(QRect(QPoint(temp.x1,temp.y1),QPoint(temp.x2,temp.y2)));

         }
         if(temp.class_id=="room")
         {
            RoomRect tempRoom;
            tempRoom.roomRect=QRect(QPoint(temp.x1,temp.y1),QPoint(temp.x2,temp.y2));
            tempRoom.roomName=QString(temp.entity_name.c_str());
            rooms.push_back(tempRoom);


         }
         if(temp.class_id=="furniture")
             notRooms.push_back(QRect(QPoint(temp.x1,temp.y1),QPoint(temp.x2,temp.y2)));

     }
     qDebug("lines:%d",lines.size());
     qDebug("rects:%d",rects.size());
     qDebug("rooms:%d",rooms.size());
     qDebug("not room rects:%d",notRooms.size());
     update();





 }
void WEImageWidget::setWorkType(WorkType &type)
{
    workType = type;
    update();
}
