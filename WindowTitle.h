#ifndef WINDOWTITLE_H
#define WINDOWTITLE_H

#include <QMouseEvent>
#include <QWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QPainter>

class WindowTitle : public QWidget
{
    Q_OBJECT
public:
    explicit WindowTitle(QWidget *parent = nullptr);
    ~WindowTitle();

    void setTitle(const QString &value);
    QPushButton* getMaxBtn() const;

signals:
    void SIG_close();   //关闭信号
    void SIG_minimize();//最小化信号
    void SIG_maximize();//最大化信号
    void SIG_normal();  //恢复信号
    void SIG_move(const QPoint & point,bool is_LeftBut);//移动距离信号，is_LeftBut：1左键mousePress，0左键mouseMove

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
private:
    bool m_isMaximized = false;
    QLabel *title;
    QPushButton *maxBtn;

private slots:
    void onCloseClicked(); //关闭槽
    void onMinimizeClicked();//最小化槽
    void onMaximizeClicked();//最大化和恢复槽
};

/*------------------------调用-----------------------
    setWindowFlags(Qt::FramelessWindowHint | Qt::Window);
    connect(ui->widget, &WindowTitle::SIG_minimize,this,[this](){
        this->showMinimized();
    });
    connect(ui->widget, &WindowTitle::SIG_maximize,this,[this](){
        this->showMaximized();
    });
    connect(ui->widget, &WindowTitle::SIG_normal,this,[this](){
        this->showNormal();
    });
    connect(ui->widget, &WindowTitle::SIG_close,this,[this](){
        this->close();
    });
    connect(ui->widget, &WindowTitle::SIG_move,this,[this](QPoint point,bool isBut){
        if(isBut){
            m_dragPosition = point - frameGeometry().topLeft();
        }else{
            if((point - m_dragPosition).manhattanLength() < 5){return;}//过滤掉双击鼠标抖动
            this->move(point - m_dragPosition);
        }

    });

---------------------------------------------------*/
#endif // WINDOWTITLE_H
