#include <QtGui>
#include "WEMapDialog.h"
WEMapDialog::WEMapDialog(QWidget *parent)
    :QDialog(parent)
{
    setupUi(this);
    name = NULL;

    connect(typeBox,SIGNAL(currentIndexChanged(QString)),this,SLOT(setNames(QString)));
}

void WEMapDialog::initDialog(QMap<QString ,QVector<QString> > &names)
{
    typeBox->addItems(names.keys());
    this->name = &names;
}

void WEMapDialog::showDialog(QRect &rect,WorkType &type)
{
    if(type == ERASER||type==RESET)
        return;
    setType(type);
    startx->setText(QString("%1").arg(rect.topLeft().x()));
    starty->setText(QString("%1").arg(rect.topLeft().y()));
    endx->setText(QString("%1").arg(rect.bottomRight().x()));
    endy->setText(QString("%1").arg(rect.bottomRight().y()));
    doorGroup->setDisabled(true);

    this->show();
    this->raise();
    this->activateWindow();
}

void WEMapDialog::showDialog(QLine &line,WorkType &type,QVector<RoomRect> &rooms)
{
    if(type == ADD_LINE)
        return;
    setType(type);
    startx->setText(QString("%1").arg(line.p1().x()));
    starty->setText(QString("%1").arg(line.p1().y()));
    endx->setText(QString("%1").arg(line.p2().x()));
    endy->setText(QString("%1").arg(line.p2().y()));
    doorGroup->setEnabled(true);
    for(int i=0;i<rooms.size();i++)
        roomaBox->addItem(rooms.at(i).roomName);
    roomaBox->addItem(QString("unknown"));

    for(int i=0;i<rooms.size();i++)
        roombBox->addItem(rooms.at(i).roomName);
    roombBox->addItem(QString("unknown"));

    roomaBox->setCurrentIndex(roomaBox->findText(GetRectNamePointIn(rooms,line.p1())));
    roombBox->setCurrentIndex(roombBox->findText(GetRectNamePointIn(rooms,line.p2())));







    this->show();
    this->raise();
    this->activateWindow();
}
QString WEMapDialog::GetRectNamePointIn(QVector<RoomRect> &rooms,QPoint p)
{
    int size=rooms.size();
    int i=0;
    for(i=0;i<size;i++)
    {
        RoomRect roomRect=rooms[i];
        QRect rect=roomRect.roomRect;
        int xMax=rect.topLeft().x()>rect.bottomRight().x()?rect.topLeft().x():rect.bottomRight().x();
        int xMin=rect.topLeft().x()>rect.bottomRight().x()?rect.bottomRight().x():rect.topLeft().x();
        int yMax=rect.topLeft().y()>rect.bottomRight().y()?rect.topLeft().y():rect.bottomRight().y();
        int yMin=rect.topLeft().y()>rect.bottomRight().y()?rect.bottomRight().y():rect.topLeft().y();

        if((p.x()<xMax && p.x()>xMin) && (p.y()>yMin && p.y()<yMax))
        {
               return roomRect.roomName;

        }




    }
    if(i>=size)
        return QString("unknown");

}
void WEMapDialog::on_okButton_clicked()
{
    if(doorGroup->isEnabled() && roomaBox->currentText() == roombBox->currentText())
        return;

    QString type = typeBox->currentText();
    QString name = nameBox->currentText();
    if (name==QString(""))
        return;
    QString tempName=name;
    if(workType == ROOM ||workType == FURNITURE)
    {

        QRect rect(QPoint(startx->text().toInt(), starty->text().toInt()), QPoint(endx->text().toInt(), endy->text().toInt()));
        emit saveRect(type, name, rect);

    }
    else if(workType == DOOR || workType == PASSWAY)
    {


       if(roomaBox->currentText()==QString("unknown")||roombBox->currentText()==QString("unknown"))
           return;






       name=name+QString(":")+roomaBox->currentText()+QString(",")+roombBox->currentText()+QString(",");

       if(workType==DOOR)
       {
           int p1x=startx->text().toAscii().toInt();
           int p1y=starty->text().toAscii().toInt();
           int p2x=endx->text().toAscii().toInt();
           int p2y=endy->text().toAscii().toInt();
           if((p2x>p1x&&p2y>p1y)||(p1x>p2x&&p1y<p2y))
               name=name+QString("left");
           else
               name=name+QString("right");





       }
       else
           name=name+QString("uncare");




        QLine line(QPoint(startx->text().toAscii().toInt(), starty->text().toAscii().toInt()),QPoint(endx->text().toAscii().toInt(),endy->text().toAscii().toInt()));
        emit saveLine(type,name, line);
    }
    //remove the name from the vector
    (*(this->name))[type].remove((*(this->name))[type].indexOf(tempName));
    setNames(type);
    this->hide();
}

void WEMapDialog::setNames(const QString &type)
{
    if(name == NULL)
        return;
    nameBox->clear();

    QVector<QString> nameList = (*name)[type];
    for(int i = 0; i < nameList.size(); i++)
    {
        nameBox->addItem(nameList[i]);
    }
}

void WEMapDialog::setType(WorkType &type)
{
    this->workType = type;
    switch(type)
    {
    case ROOM:
        typeBox->setCurrentIndex(typeBox->findText("room"));
        break;
    case DOOR:
        typeBox->setCurrentIndex(typeBox->findText("door"));
        break;
    case PASSWAY:
        typeBox->setCurrentIndex(typeBox->findText("passway"));
        break;
    case FURNITURE:
        typeBox->setCurrentIndex(typeBox->findText("furniture"));
        break;
    }
}



