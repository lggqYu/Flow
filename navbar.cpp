//#pragma execution_character_set("utf-8")

#include "navbar.h"
#include "qevent.h"
#include "qpainter.h"
#include <QPainterPath>
#include "qtimer.h"
#include "qdebug.h"
#include<QLabel>
#include <QPainter>
#include <QWidget>// 转换函数定义
#include <QFontDatabase>
#include <QHoverEvent>

NavBar::NavBar(QWidget *parent) : QWidget(parent)
{
    // 加载自定义字体
    QFontDatabase::addApplicationFont(":/image/ALIBABA-PUHUITI-BOLD.OTF"); // 指定字体路径
    // 获取字体族名
    FontName = "AlibabaPuHuiTiB"; // 使用字体名称
    setMouseTracking(true);  // 启用鼠标追踪
    setAttribute(Qt::WA_Hover); // 启用 Hover 悬停事件

    bgColorStart = QColor(63, 71, 101);// navbar外层背景颜色
    bgColorEnd = QColor(63, 71, 101);
    inColorStart = QColor(130, 135, 155, 210);// navbar内层阴影颜色
    inColorEnd = QColor(130, 135, 155, 40);
    old_bgColorEnd = bgColorEnd;

    barColorStart = QColor(2, 130, 189);
    barColorEnd = QColor(19, 173, 159);
    old_barColorEnd = barColorEnd;

    HoverColorStart = QColor(30,35,55);// navbar悬停颜色
    HoverColorEnd = QColor(30,35,55);

    textNormalColor = QColor(255, 255, 255);
    textSelectColor = QColor(255, 255, 255);

    items = "";
    currentIndex = -1;
    currentHoverIndex = -1;
    currentItem = "";

    bgRadius = 31;
    barRadius = 28;
    HoverRadius = 28;
    space = 25;

    lineWidth = 3;
    lineColor = QColor(13, 128, 203);

    barStyle = BarStyle_Line_Bottom;

    keyMove = false;
    horizontal = true;
    flat = false;

    initLen = 10;
    step = 0;
    textLen = 0;

    isForward = true;
    isVirgin = true;

    timer = new QTimer(this);
    timer->setInterval(10);
    connect(timer, SIGNAL(timeout()), this, SLOT(slide()));
}

NavBar::~NavBar()
{
    if (timer->isActive()) {
        timer->stop();
    }
}

void NavBar::resizeEvent(QResizeEvent *)
{
    int index = 0;
    int count = listItem.count();
    if (count == 0) {
        return;
    }

    if (count > 0 && currentItem.isEmpty()) {
        currentIndex = 0;
        currentItem = listItem.at(0).first;
    }

    for (int i = 0; i < count; i++) {
        if (listItem.at(i).first == currentItem) {
            index = i;
            break;
        }
    }

    moveTo(index);
}

void NavBar::mousePressEvent(QMouseEvent *e)
{
//    moveTo(e->pos());
//    emit inputChanged(1);


    QPointF clickPos = e->pos(); // 获取鼠标点击的位置
        int count = listItem.count();

        for (int i = 0; i < count; i++) {
            if (listItem.at(i).second.contains(clickPos)) {
                currentIndex = i; // 更新当前索引
                currentItem = listItem.at(i).first; // 更新当前条目
                moveTo(currentIndex); // 移动到对应的条目
                emit signalItemCutPage(currentIndex); // 发出不同的信号
                break; // 找到对应条目后退出循环
            }
        }
}

void NavBar::keyPressEvent(QKeyEvent *keyEvent)
{
    if (!keyMove) {
        return;
    }

    if (keyEvent->key() == Qt::Key_Left || keyEvent->key() == Qt::Key_Up) {
        movePrevious();
    } else if (keyEvent->key() == Qt::Key_Right || keyEvent->key() == Qt::Key_Down) {
        moveNext();
    }
}

void NavBar::paintEvent(QPaintEvent *)
{
    //绘制准备工作,启用反锯齿
    QPainter painter(this);
    painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);

    //绘制导航条背景色
    drawBg(&painter);
    //绘制悬停条目选中背景
    drawHover(&painter);
    //绘制当前条目选中背景
    drawBar(&painter);

    //绘制条目文字
    drawText(&painter);
}

void NavBar::drawBg(QPainter *painter)
{
    painter->save();

    // 绘制外层背景渐变
    QLinearGradient bgGradient(QPoint(0, 0), QPoint(0, height()));
    bgGradient.setColorAt(0.0, bgColorStart);
    bgGradient.setColorAt(1.0, bgColorEnd);
    painter->setBrush(bgGradient);
    painter->setPen(QPen(QColor(59, 65, 99), 1)); // 添加外层描边
    painter->drawRoundedRect(rect(), bgRadius, bgRadius);
    // 绘制内层阴影渐变
    QRect innerRect = rect().adjusted(8, 8, -8, -8);
    painter->setPen(Qt::NoPen);
    QRadialGradient shadowGradient(innerRect.center(), 11, innerRect.center());
    shadowGradient.setColorAt(0.0, inColorStart);
    shadowGradient.setColorAt(1.0, inColorEnd);
    painter->setBrush(shadowGradient);
    painter->setRenderHint(QPainter::Antialiasing);

    painter->drawRoundedRect(8, 8, width() - 16, height() - 16, 21, 21); // 绘制内阴影，扩散 8px

    painter->restore();
}


void NavBar::drawBar(QPainter *painter)//当前条目
{
    painter->save();
    QPen pen;
    QLinearGradient barGradient(barRect.topLeft(), barRect.topRight());
    barGradient.setColorAt(0.0, barColorStart);
    barGradient.setColorAt(1.0, barColorEnd);
    painter->setBrush(barGradient);

    painter->setPen(Qt::NoPen);
    painter->drawRoundedRect(barRect, barRadius, barRadius);
    painter->restore();
}

void NavBar::drawText(QPainter *painter)
{
    painter->save();

    QFont textFont;
    textFont.setBold(true);
    textFont.setPointSize(19);
    textFont.setFamily(FontName);
    painter->setFont(textFont);

    int count = listItem.count();
    QString strText;
    initLen = 0;


    //横向导航时,字符区域取条目元素中最长的字符宽度
    QString longText = "";
    QStringList list = items.split(";");
    foreach (QString str, list) {
        if (str.length() > longText.length()) {
            longText = str;
        }
    }

    if (horizontal) {
        textLen  = width()/count;       /*painter->fontMetrics().width(longText)*/
    } else {
        textLen  = height()/count;      /*painter->fontMetrics().height()*/
    }

    //逐个绘制元素队列中的文字及文字背景
    for (int i = 0; i < count; i++) {
        strText = listItem.at(i).first;
        QPointF left(initLen, 0);
        QPointF right(/*initLen + textLen + space*/    initLen + textLen, height());

        if (!horizontal) {
            left = QPointF(0, initLen);
            right = QPointF(width(), /*initLen + textLen + space*/    initLen + textLen);
        }

        QRectF textRect(left, right);
        listItem[i].second = textRect;

        if (isVirgin) {
            barRect = textRect;
            isVirgin = false;
        }

        //当前选中区域的文字显示选中文字颜色
        if (textRect == listItem.at(currentIndex).second) {
            painter->setPen(textSelectColor);
        } else {
            painter->setPen(textNormalColor);
        }

        painter->drawText(textRect, Qt::AlignCenter, strText);
        initLen += textLen/* + space*/;
    }
    painter->restore();
}

void NavBar::drawHover(QPainter *painter)
{
    if (currentHoverIndex != -1)
    {
        HoverRect = listItem[currentHoverIndex].second; // 获取当前悬停的矩形区域
        painter->save();
        QPen pen;
        QLinearGradient barGradient(HoverRect.topLeft(), HoverRect.bottomLeft());
        barGradient.setColorAt(0.0, HoverColorStart);
        barGradient.setColorAt(1.0, HoverColorEnd);
        painter->setBrush(barGradient);

        painter->setPen(Qt::NoPen);
        painter->drawRoundedRect(HoverRect, HoverRadius, HoverRadius);
        painter->restore();
    }
}

bool NavBar::event(QEvent *event)
{
    if (event->type() == QEvent::HoverMove) // 处理鼠标移动时的悬停事件
    {
        QHoverEvent *hoverEvent = static_cast<QHoverEvent *>(event);
        QPoint hoverPos = hoverEvent->pos();  // 获取悬停位置

        int hoverIndex = -1;
        // 遍历 listItem，判断鼠标悬停在哪个文字上
        for (int i = 0; i < listItem.size(); ++i)
        {
            if (listItem[i].second.contains(hoverPos)) // 检查悬停位置是否在某个文本区域内
            {
                hoverIndex = i;
                break;
            }
        }

        // 如果悬停的文字有变化，更新 HoverRect 并重新绘制
        if (hoverIndex != currentHoverIndex)
        {
            currentHoverIndex = hoverIndex; // 更新悬停索引
            update(); // 触发重绘
        }
        return true; // 表示事件已处理
    }

    // 处理鼠标离开事件
    if (event->type() == QEvent::HoverLeave)
    {
        if (currentHoverIndex != -1) // 如果有悬停索引
        {
            currentHoverIndex = -1;  // 重置悬停索引
            update(); // 触发重绘
        }
        return true; // 表示事件已处理
    }
    return QWidget::event(event); // 交给基类处理其他事件
}

int NavBar::initStep(int distance)
{
    int n = 1;

    while (1) {
        if (n * n > distance) {
            break;
        } else {
            n++;
        }
    }

    return n * 1.4;
}

void NavBar::slide()
{
    if (step > 1) {
        step--;
    }

    if (horizontal) {
        barLen = barRect.topLeft().x();
    } else {
        barLen = barRect.topLeft().y();
    }

    if (isForward) {
        barLen += step;
        if (barLen >= targetLen) {
            barLen = targetLen;
            timer->stop();
        }
    } else {
        barLen -= step;
        if (barLen <= targetLen) {
            barLen = targetLen;
            timer->stop();
        }
    }

    if (horizontal) {
        barRect = QRectF(QPointF(barLen, 0), QPointF(barLen + /*barRect.width()*/textLen, height()));
    } else {
        barRect = QRectF(QPointF(0, barLen), QPointF(width(), barLen + textLen/*barRect.height()*/));
    }

    this->update();
}

QColor NavBar::getBgColorStart() const
{
    return this->bgColorStart;
}

QColor NavBar::getBgColorEnd() const
{
    return this->bgColorEnd;
}

QColor NavBar::getBarColorStart() const
{
    return this->barColorStart;
}

QColor NavBar::getBarColorEnd() const
{
    return this->barColorEnd;
}

QColor NavBar::getTextNormalColor() const
{
    return this->textNormalColor;
}

QColor NavBar::getTextSelectColor() const
{
    return this->textSelectColor;
}

QString NavBar::getItems() const
{
    return this->items;
}

int NavBar::getCurrentIndex() const
{
    return this->currentIndex;
}

QString NavBar::getCurrentItem() const
{
    return this->currentItem;
}

int NavBar::getBgRadius() const
{
    return this->bgRadius;
}

int NavBar::getBarRadius() const
{
    return this->barRadius;
}

int NavBar::getSpace() const
{
    return this->space;
}

int NavBar::getLineWidth() const
{
    return this->lineWidth;
}

QColor NavBar::getLineColor() const
{
    return this->lineColor;
}

NavBar::BarStyle NavBar::getBarStyle() const
{
    return this->barStyle;
}

bool NavBar::getKeyMove() const
{
    return this->keyMove;
}

bool NavBar::getHorizontal() const
{
    return this->horizontal;
}

bool NavBar::getFlat() const
{
    return this->flat;
}

QSize NavBar::sizeHint() const
{
    return QSize(500, 30);
}

QSize NavBar::minimumSizeHint() const
{
    return QSize(30, 30);
}

void NavBar::clearItem()
{
    listItem.clear();
    this->update();
}

void NavBar::setBgColorStart(const QColor &bgColorStart)
{
    if (this->bgColorStart != bgColorStart) {
        this->bgColorStart = bgColorStart;
        this->update();
    }
}

void NavBar::setBgColorEnd(const QColor &bgColorEnd)
{
    if (this->bgColorEnd != bgColorEnd) {
        this->bgColorEnd = bgColorEnd;
        this->old_bgColorEnd = bgColorEnd;
        this->update();
    }
}

void NavBar::setBarColorStart(const QColor &barColorStart)
{
    if (this->barColorStart != barColorStart) {
        this->barColorStart = barColorStart;
        this->update();
    }
}

void NavBar::setBarColorEnd(const QColor &barColorEnd)
{
    if (this->barColorEnd != barColorEnd) {
        this->barColorEnd = barColorEnd;
        this->old_barColorEnd = barColorEnd;
        this->update();
    }
}

void NavBar::setTextNormalColor(const QColor &textNormalColor)
{
    if (this->textNormalColor != textNormalColor) {
        this->textNormalColor = textNormalColor;
        this->update();
    }
}

void NavBar::setTextSelectColor(const QColor &textSelectColor)
{
    if (this->textSelectColor != textSelectColor) {
        this->textSelectColor = textSelectColor;
        this->update();
    }
}

void NavBar::setItems(const QString &items)
{
    this->items = items;
    this->listItem.clear();

    QStringList list = items.split(";");
    foreach (QString str, list) {
        this->listItem.push_back(qMakePair(str, QRectF()));
    }
    this->update();
}

void NavBar::setCurrentIndex(int index)
{
    moveTo(index);
}

void NavBar::setCurrentItem(const QString &item)
{
    moveTo(item);
}

void NavBar::setBgRadius(int bgRadius)
{
    if (this->bgRadius != bgRadius) {
        this->bgRadius = bgRadius;
        this->update();
    }
}

void NavBar::setBarRadius(int barRadius)
{
    if (this->barRadius != barRadius) {
        this->barRadius = barRadius;
        this->update();
    }
}

void NavBar::setSpace(int space)
{
    if (this->space != space) {
        this->space = space;
        this->update();
    }
}

void NavBar::setLineWidth(int lineWidth)
{
    if (this->lineWidth != lineWidth) {
        this->lineWidth = lineWidth;
        this->update();
    }
}

void NavBar::setLineColor(const QColor &lineColor)
{
    if (this->lineColor != lineColor) {
        this->lineColor = lineColor;
        this->update();
    }
}

void NavBar::setBarStyle(const NavBar::BarStyle &barStyle)
{
    if (this->barStyle != barStyle) {
        this->barStyle = barStyle;
        this->update();
    }
}

void NavBar::setKeyMove(bool keyMove)
{
    if (this->keyMove != keyMove) {
        this->keyMove = keyMove;
        if (keyMove) {
            setFocusPolicy(Qt::StrongFocus);
        } else {
            setFocusPolicy(Qt::NoFocus);
        }
    }
}

void NavBar::setHorizontal(bool horizontal)
{
    if (this->horizontal != horizontal) {
        this->horizontal = horizontal;
        this->update();
    }
}

void NavBar::setFlat(bool flat)
{
    if (this->flat != flat) {
        //扁平后将初始颜色赋值给结束颜色达到扁平的效果,如果取消扁平则再次恢复原有的颜色
        if (flat) {
            bgColorEnd = bgColorStart;
            barColorEnd = barColorStart;
        } else {
            bgColorEnd = old_bgColorEnd;
            barColorEnd = old_barColorEnd;
        }

        this->flat = flat;
        this->update();
    }
}

void NavBar::moveFirst()
{
    int index = 0;
    if (currentIndex != index) {
        moveTo(index);
    }
}

void NavBar::moveLast()
{
    int index = listItem.count() - 1;
    if (currentIndex != index) {
        moveTo(index);
    }
}

void NavBar::movePrevious()
{
    if (currentIndex > 0) {
        currentIndex--;
        moveTo(currentIndex);
    }
}

void NavBar::moveNext()
{
    if (currentIndex < listItem.count() - 1) {
        currentIndex++;
        moveTo(currentIndex);
    }
}

void NavBar::moveTo(int index)
{
    if (index >= 0 && listItem.count() > index) {
        QRectF rec = listItem.at(index).second;
        QPoint pos = QPoint(rec.x(), rec.y());
        moveTo(pos);
    }
}

void NavBar::moveTo(const QString &item)
{
    int count = listItem.count();
    for (int i = 0; i < count; i++) {
        if (listItem.at(i).first == item) {
            moveTo(i);
            break;
        }
    }
}

void NavBar::moveTo(const QPointF &point)
{
    int count = listItem.count();
    for (int i = 0; i < count; i++) {
        if (listItem.at(i).second.contains(point)) {
            currentIndex = i;
            currentItem = listItem.at(i).first;
            targetRect = listItem.at(i).second;

            if (horizontal) {
                targetLen = targetRect.topLeft().x();
                barLen = barRect.topLeft().x();
            } else {
                targetLen = targetRect.topLeft().y();
                barLen = barRect.topLeft().y();
            }

            isForward = (targetLen > barLen);
            int distance = targetLen - barLen;
            distance = qAbs(distance);

            //重新获取每次移动的步长
            step = initStep(distance);
            timer->start();
            emit currentItemChanged(currentIndex, currentItem);
        }
    }
}
