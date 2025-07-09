#include "WindowTitle.h"

WindowTitle::WindowTitle(QWidget *parent) : QWidget(parent)
{
    setAttribute(Qt::WA_StyledBackground, true);
    setFixedHeight(30);
    title = new QLabel("Window");
    QPushButton *minBtn = new QPushButton();
    QPushButton *closeBtn = new QPushButton();
    maxBtn = new QPushButton();
    minBtn->setFixedSize(30, 25);
    closeBtn->setFixedSize(30, 25);
    maxBtn->setFixedSize(30, 25);
    minBtn->setIcon(QIcon(":/QtWorktitle/IconRes/win_minimize.png"));
    closeBtn->setIcon(QIcon(":/QtWorktitle/IconRes/win_close.png"));
    maxBtn->setIcon(QIcon(":/QtWorktitle/IconRes/win_maximize.png"));

    QHBoxLayout *titleLayout = new QHBoxLayout();
    titleLayout->addWidget(title);
    titleLayout->addStretch();
    titleLayout->addWidget(minBtn);
    titleLayout->addWidget(maxBtn);
    titleLayout->addWidget(closeBtn);
    titleLayout->setContentsMargins(5, 0, 5, 0);
    setLayout(titleLayout);

    connect(minBtn, &QPushButton::clicked, this, &WindowTitle::onMinimizeClicked);
    connect(maxBtn, &QPushButton::clicked, this, &WindowTitle::onMaximizeClicked);
    connect(closeBtn, &QPushButton::clicked, this, &WindowTitle::onCloseClicked);
}

WindowTitle::~WindowTitle()
{
    delete maxBtn;
    delete title;
}

void WindowTitle::setTitle(const QString &value)
{
    title->setText(value);
}

void WindowTitle::onCloseClicked()
{
    emit SIG_close();
}

void WindowTitle::onMinimizeClicked()
{
    emit SIG_minimize();
}

void WindowTitle::onMaximizeClicked()
{
//    QPushButton *maxBtn = qobject_cast<QPushButton *>(sender());
    if (m_isMaximized) {
        emit SIG_normal();
        m_isMaximized = false;
        maxBtn->setIcon(QIcon(":/QtWorktitle/IconRes/win_maximize.png"));
    } else {
        emit SIG_maximize();
        m_isMaximized = true;
        maxBtn->setIcon(QIcon(":/QtWorktitle/IconRes/win_normal.png"));
    }
}

// 拖动窗口
void WindowTitle::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton){
        emit SIG_move(event->globalPos(),true);
    }
    QWidget::mousePressEvent(event);
}

void WindowTitle::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton && !m_isMaximized){
        emit SIG_move(event->globalPos(),false);
    }
    QWidget::mouseMoveEvent(event);
}

void WindowTitle::mouseDoubleClickEvent(QMouseEvent *event) {
    Q_UNUSED(event);
    onMaximizeClicked();
}

QPushButton* WindowTitle::getMaxBtn() const
{
    return maxBtn;
}
