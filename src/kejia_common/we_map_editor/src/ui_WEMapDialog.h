/********************************************************************************
** Form generated from reading UI file 'WEMapDialog.ui'
**
** Created: Tue Apr 16 15:15:59 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WEMAPDIALOG_H
#define UI_WEMAPDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MapDialog
{
public:
    QWidget *widget;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout_3;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QComboBox *typeBox;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QComboBox *nameBox;
    QHBoxLayout *horizontalLayout_7;
    QGroupBox *groupBox;
    QWidget *layoutWidget_2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_3;
    QLineEdit *startx;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_5;
    QLineEdit *starty;
    QSpacerItem *horizontalSpacer_3;
    QGroupBox *groupBox_2;
    QWidget *layoutWidget_5;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_7;
    QLineEdit *endx;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_8;
    QLineEdit *endy;
    QSpacerItem *verticalSpacer;
    QGroupBox *doorGroup;
    QWidget *widget1;
    QHBoxLayout *horizontalLayout_12;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_4;
    QComboBox *roomaBox;
    QSpacerItem *horizontalSpacer_6;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_6;
    QComboBox *roombBox;
    QSpacerItem *verticalSpacer_2;
    QHBoxLayout *horizontalLayout_13;
    QSpacerItem *horizontalSpacer_4;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *okButton;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *cancelButton;
    QSpacerItem *horizontalSpacer_5;

    void setupUi(QDialog *MapDialog)
    {
        if (MapDialog->objectName().isEmpty())
            MapDialog->setObjectName(QString::fromUtf8("MapDialog"));
        MapDialog->resize(445, 299);
        MapDialog->setMinimumSize(QSize(0, 0));
        widget = new QWidget(MapDialog);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(10, 10, 423, 273));
        gridLayout = new QGridLayout(widget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(widget);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        typeBox = new QComboBox(widget);
        typeBox->setObjectName(QString::fromUtf8("typeBox"));
        typeBox->setMinimumSize(QSize(125, 0));
        typeBox->setIconSize(QSize(16, 16));

        horizontalLayout->addWidget(typeBox);


        horizontalLayout_3->addLayout(horizontalLayout);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_2 = new QLabel(widget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_2->addWidget(label_2);

        nameBox = new QComboBox(widget);
        nameBox->setObjectName(QString::fromUtf8("nameBox"));
        nameBox->setMinimumSize(QSize(125, 27));

        horizontalLayout_2->addWidget(nameBox);


        horizontalLayout_3->addLayout(horizontalLayout_2);


        gridLayout->addLayout(horizontalLayout_3, 0, 0, 1, 2);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        groupBox = new QGroupBox(widget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setMinimumSize(QSize(171, 101));
        layoutWidget_2 = new QWidget(groupBox);
        layoutWidget_2->setObjectName(QString::fromUtf8("layoutWidget_2"));
        layoutWidget_2->setGeometry(QRect(0, 30, 166, 66));
        verticalLayout = new QVBoxLayout(layoutWidget_2);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_3 = new QLabel(layoutWidget_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_4->addWidget(label_3);

        startx = new QLineEdit(layoutWidget_2);
        startx->setObjectName(QString::fromUtf8("startx"));
        startx->setMinimumSize(QSize(146, 27));
        startx->setMaximumSize(QSize(146, 27));

        horizontalLayout_4->addWidget(startx);


        verticalLayout->addLayout(horizontalLayout_4);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_5 = new QLabel(layoutWidget_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_6->addWidget(label_5);

        starty = new QLineEdit(layoutWidget_2);
        starty->setObjectName(QString::fromUtf8("starty"));
        starty->setMinimumSize(QSize(146, 27));
        starty->setMaximumSize(QSize(146, 27));

        horizontalLayout_6->addWidget(starty);


        verticalLayout->addLayout(horizontalLayout_6);


        horizontalLayout_7->addWidget(groupBox);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_3);

        groupBox_2 = new QGroupBox(widget);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setMinimumSize(QSize(171, 101));
        layoutWidget_5 = new QWidget(groupBox_2);
        layoutWidget_5->setObjectName(QString::fromUtf8("layoutWidget_5"));
        layoutWidget_5->setGeometry(QRect(0, 30, 166, 66));
        verticalLayout_2 = new QVBoxLayout(layoutWidget_5);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        label_7 = new QLabel(layoutWidget_5);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        horizontalLayout_8->addWidget(label_7);

        endx = new QLineEdit(layoutWidget_5);
        endx->setObjectName(QString::fromUtf8("endx"));
        endx->setMinimumSize(QSize(146, 27));
        endx->setMaximumSize(QSize(146, 27));

        horizontalLayout_8->addWidget(endx);


        verticalLayout_2->addLayout(horizontalLayout_8);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_8 = new QLabel(layoutWidget_5);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout_9->addWidget(label_8);

        endy = new QLineEdit(layoutWidget_5);
        endy->setObjectName(QString::fromUtf8("endy"));
        endy->setMinimumSize(QSize(146, 27));
        endy->setMaximumSize(QSize(146, 27));

        horizontalLayout_9->addWidget(endy);


        verticalLayout_2->addLayout(horizontalLayout_9);


        horizontalLayout_7->addWidget(groupBox_2);


        gridLayout->addLayout(horizontalLayout_7, 1, 0, 1, 2);

        verticalSpacer = new QSpacerItem(20, 5, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer, 2, 0, 1, 1);

        doorGroup = new QGroupBox(widget);
        doorGroup->setObjectName(QString::fromUtf8("doorGroup"));
        doorGroup->setMinimumSize(QSize(421, 61));
        widget1 = new QWidget(doorGroup);
        widget1->setObjectName(QString::fromUtf8("widget1"));
        widget1->setGeometry(QRect(10, 20, 391, 31));
        horizontalLayout_12 = new QHBoxLayout(widget1);
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        horizontalLayout_12->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        label_4 = new QLabel(widget1);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_10->addWidget(label_4);

        roomaBox = new QComboBox(widget1);
        roomaBox->setObjectName(QString::fromUtf8("roomaBox"));
        roomaBox->setMinimumSize(QSize(125, 27));

        horizontalLayout_10->addWidget(roomaBox);


        horizontalLayout_12->addLayout(horizontalLayout_10);

        horizontalSpacer_6 = new QSpacerItem(10, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_12->addItem(horizontalSpacer_6);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_6 = new QLabel(widget1);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        horizontalLayout_11->addWidget(label_6);

        roombBox = new QComboBox(widget1);
        roombBox->setObjectName(QString::fromUtf8("roombBox"));
        roombBox->setMinimumSize(QSize(125, 27));

        horizontalLayout_11->addWidget(roombBox);


        horizontalLayout_12->addLayout(horizontalLayout_11);


        gridLayout->addWidget(doorGroup, 3, 0, 1, 2);

        verticalSpacer_2 = new QSpacerItem(20, 10, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer_2, 4, 1, 1, 1);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_13->addItem(horizontalSpacer_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        okButton = new QPushButton(widget);
        okButton->setObjectName(QString::fromUtf8("okButton"));
        okButton->setDefault(true);

        horizontalLayout_5->addWidget(okButton);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_2);

        cancelButton = new QPushButton(widget);
        cancelButton->setObjectName(QString::fromUtf8("cancelButton"));

        horizontalLayout_5->addWidget(cancelButton);


        horizontalLayout_13->addLayout(horizontalLayout_5);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_13->addItem(horizontalSpacer_5);


        gridLayout->addLayout(horizontalLayout_13, 5, 0, 1, 2);

#ifndef QT_NO_SHORTCUT
        label->setBuddy(typeBox);
        label_2->setBuddy(nameBox);
        label_4->setBuddy(roomaBox);
        label_6->setBuddy(roombBox);
#endif // QT_NO_SHORTCUT
        QWidget::setTabOrder(typeBox, okButton);
        QWidget::setTabOrder(okButton, startx);
        QWidget::setTabOrder(startx, starty);
        QWidget::setTabOrder(starty, endx);
        QWidget::setTabOrder(endx, endy);
        QWidget::setTabOrder(endy, cancelButton);

        retranslateUi(MapDialog);
        QObject::connect(cancelButton, SIGNAL(clicked()), MapDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(MapDialog);
    } // setupUi

    void retranslateUi(QDialog *MapDialog)
    {
        MapDialog->setWindowTitle(QApplication::translate("MapDialog", "Topo Map Dialog", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MapDialog", "&Type", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MapDialog", "&Name", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MapDialog", "Start Point", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MapDialog", "X", 0, QApplication::UnicodeUTF8));
        startx->setText(QString());
        label_5->setText(QApplication::translate("MapDialog", "Y", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("MapDialog", "End Point", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MapDialog", "X", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MapDialog", "Y", 0, QApplication::UnicodeUTF8));
        doorGroup->setTitle(QApplication::translate("MapDialog", "Door Info", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MapDialog", "Room&A", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MapDialog", "Room&B", 0, QApplication::UnicodeUTF8));
        okButton->setText(QApplication::translate("MapDialog", "&OK", 0, QApplication::UnicodeUTF8));
        cancelButton->setText(QApplication::translate("MapDialog", "&Cancel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MapDialog: public Ui_MapDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WEMAPDIALOG_H
