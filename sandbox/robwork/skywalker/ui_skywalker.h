/********************************************************************************
** Form generated from reading UI file 'skywalker.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SKYWALKER_H
#define UI_SKYWALKER_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDockWidget>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SkywalkerWidget
{
public:
    QWidget *dockWidgetContents;
    QGridLayout *gridLayout;
    QFrame *line;
    QPushButton *restoreButton;
    QPushButton *saveButton;
    QCheckBox *enableBox;
    QSpacerItem *verticalSpacer;
    QPushButton *snapshotButton;

    void setupUi(QDockWidget *SkywalkerWidget)
    {
        if (SkywalkerWidget->objectName().isEmpty())
            SkywalkerWidget->setObjectName(QString::fromUtf8("SkywalkerWidget"));
        SkywalkerWidget->resize(400, 300);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        gridLayout = new QGridLayout(dockWidgetContents);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        line = new QFrame(dockWidgetContents);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        gridLayout->addWidget(line, 2, 0, 1, 2);

        restoreButton = new QPushButton(dockWidgetContents);
        restoreButton->setObjectName(QString::fromUtf8("restoreButton"));

        gridLayout->addWidget(restoreButton, 1, 1, 1, 1);

        saveButton = new QPushButton(dockWidgetContents);
        saveButton->setObjectName(QString::fromUtf8("saveButton"));

        gridLayout->addWidget(saveButton, 1, 0, 1, 1);

        enableBox = new QCheckBox(dockWidgetContents);
        enableBox->setObjectName(QString::fromUtf8("enableBox"));

        gridLayout->addWidget(enableBox, 0, 0, 1, 2);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer, 4, 0, 1, 1);

        snapshotButton = new QPushButton(dockWidgetContents);
        snapshotButton->setObjectName(QString::fromUtf8("snapshotButton"));

        gridLayout->addWidget(snapshotButton, 3, 0, 1, 2);

        SkywalkerWidget->setWidget(dockWidgetContents);

        retranslateUi(SkywalkerWidget);

        QMetaObject::connectSlotsByName(SkywalkerWidget);
    } // setupUi

    void retranslateUi(QDockWidget *SkywalkerWidget)
    {
        SkywalkerWidget->setWindowTitle(QApplication::translate("SkywalkerWidget", "Skywalker Plugin", 0, QApplication::UnicodeUTF8));
        restoreButton->setText(QApplication::translate("SkywalkerWidget", "Restore position", 0, QApplication::UnicodeUTF8));
        saveButton->setText(QApplication::translate("SkywalkerWidget", "Save position", 0, QApplication::UnicodeUTF8));
        enableBox->setText(QApplication::translate("SkywalkerWidget", "Enabled", 0, QApplication::UnicodeUTF8));
        snapshotButton->setText(QApplication::translate("SkywalkerWidget", "Snapshot", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SkywalkerWidget: public Ui_SkywalkerWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SKYWALKER_H
