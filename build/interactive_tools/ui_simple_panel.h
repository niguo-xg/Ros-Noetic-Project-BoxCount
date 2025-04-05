/********************************************************************************
** Form generated from reading UI file 'simple_panel.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SIMPLE_PANEL_H
#define UI_SIMPLE_PANEL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TaskControlPanel
{
public:
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QLabel *label_status;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButton_regen;
    QPushButton *pushButton_clear;

    void setupUi(QWidget *TaskControlPanel)
    {
        if (TaskControlPanel->objectName().isEmpty())
            TaskControlPanel->setObjectName(QString::fromUtf8("TaskControlPanel"));
        TaskControlPanel->setWindowModality(Qt::NonModal);
        TaskControlPanel->resize(300, 75);
        TaskControlPanel->setMinimumSize(QSize(300, 75));
        TaskControlPanel->setMaximumSize(QSize(300, 75));
        verticalLayoutWidget = new QWidget(TaskControlPanel);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 0, 281, 31));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        label_status = new QLabel(verticalLayoutWidget);
        label_status->setObjectName(QString::fromUtf8("label_status"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label_status->sizePolicy().hasHeightForWidth());
        label_status->setSizePolicy(sizePolicy);
        label_status->setMinimumSize(QSize(135, 25));
        label_status->setMaximumSize(QSize(270, 50));
        label_status->setBaseSize(QSize(100, 25));
        QFont font;
        font.setFamily(QString::fromUtf8("Ubuntu"));
        font.setPointSize(11);
        font.setBold(false);
        font.setItalic(false);
        font.setUnderline(false);
        font.setWeight(50);
        label_status->setFont(font);
        label_status->setAutoFillBackground(false);
        label_status->setAlignment(Qt::AlignCenter);
        label_status->setWordWrap(false);

        verticalLayout->addWidget(label_status);

        horizontalLayoutWidget = new QWidget(TaskControlPanel);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(10, 30, 281, 41));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        pushButton_regen = new QPushButton(horizontalLayoutWidget);
        pushButton_regen->setObjectName(QString::fromUtf8("pushButton_regen"));
        pushButton_regen->setMinimumSize(QSize(25, 25));
        pushButton_regen->setMaximumSize(QSize(150, 25));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Ubuntu"));
        font1.setBold(true);
        font1.setItalic(false);
        font1.setWeight(75);
        font1.setStrikeOut(false);
        pushButton_regen->setFont(font1);
        pushButton_regen->setMouseTracking(false);
        pushButton_regen->setAutoFillBackground(true);

        horizontalLayout->addWidget(pushButton_regen);

        pushButton_clear = new QPushButton(horizontalLayoutWidget);
        pushButton_clear->setObjectName(QString::fromUtf8("pushButton_clear"));
        pushButton_clear->setMinimumSize(QSize(25, 25));
        pushButton_clear->setMaximumSize(QSize(150, 25));
        QFont font2;
        font2.setFamily(QString::fromUtf8("Ubuntu"));
        font2.setBold(true);
        font2.setItalic(false);
        font2.setWeight(75);
        pushButton_clear->setFont(font2);
        pushButton_clear->setMouseTracking(false);
        pushButton_clear->setAutoFillBackground(true);

        horizontalLayout->addWidget(pushButton_clear);


        retranslateUi(TaskControlPanel);

        QMetaObject::connectSlotsByName(TaskControlPanel);
    } // setupUi

    void retranslateUi(QWidget *TaskControlPanel)
    {
        TaskControlPanel->setWindowTitle(QApplication::translate("TaskControlPanel", "Form", nullptr));
        label_status->setText(QApplication::translate("TaskControlPanel", "ME5413 Gazebo World Control", nullptr));
        pushButton_regen->setText(QApplication::translate("TaskControlPanel", "Respawn Objects", nullptr));
        pushButton_clear->setText(QApplication::translate("TaskControlPanel", "Clear Objects", nullptr));
    } // retranslateUi

};

namespace Ui {
    class TaskControlPanel: public Ui_TaskControlPanel {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SIMPLE_PANEL_H
