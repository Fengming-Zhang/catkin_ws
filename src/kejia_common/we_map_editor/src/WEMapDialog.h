#ifndef WEMAPDIALOG_H
#define WEMAPDIALOG_H
#include <QtGui>
#include "ui_WEMapDialog.h"
#include "WEImageWidget.h"
class WEMapDialog : public QDialog, public Ui::MapDialog
{
    Q_OBJECT

public:
    WEMapDialog(QWidget * parent = 0);
    void initDialog(QMap<QString ,QVector<QString> > &names);
    QString GetRectNamePointIn(QVector<RoomRect> &rooms,QPoint p);


signals:
    void saveRect(QString &type, QString &name, QRect &rect);
    void saveLine(QString &type, QString &name, QLine &line);

public slots:
    void showDialog(QRect &rect, WorkType &type);
    void showDialog(QLine &line, WorkType &type,QVector<RoomRect> &rooms);
    void setNames(const QString &type);
private slots:
    void on_okButton_clicked();


private:
    WorkType workType;
    void setType(WorkType &type);
    QMap<QString ,QVector<QString> > *name;


};

#endif // WEMAPDIALOG_H
