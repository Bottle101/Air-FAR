#include "drive_widget.h"
#include "ros/ros.h"
#include <algorithm>

namespace teleop_rviz_plugin
{

float z_velocity_ = 0.0;

DriveWidget::DriveWidget( QWidget* parent )
  : QWidget( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
  , linear_scale_( 1 )
  , angular_scale_( 1 )
  , x_mouse_( 0 )
  , y_mouse_( 0 )
  , mouse_pressed_(false)
{
  // Initialize the z_velocity slider
  zVelocitySlider = new QSlider(Qt::Vertical, this);
  zVelocitySlider->setRange(-100, 100);  // Assuming z_velocity_ ranges from -1.0 to 1.0, scaled by 100 for slider precision
  zVelocitySlider->setValue(z_velocity_ * 100);
  zVelocitySlider->setTickPosition(QSlider::TicksLeft);
  zVelocitySlider->setTickInterval(20);
  // zVelocitySlider->setMinimumSize(60, size);
  int size = (( width() > height() ) ? height() : width()) - 1 - 60;
  zVelocitySlider->setMaximumHeight(size);

  // Layout to add slider
  QVBoxLayout* layout = new QVBoxLayout(this);
  // layout->addStretch(1); 
  layout->addWidget(zVelocitySlider);
  setLayout(layout);

  // Connect the slider's valueChanged signal to a slot
  connect(zVelocitySlider, &QSlider::valueChanged, this, &DriveWidget::setZVelocity);
}

void DriveWidget::setZVelocity(int value)
{
    z_velocity_ = value / 100.0;
    // emit outputVelocity(linear_velocity_, angular_velocity_, mouse_pressed_, z_velocity_);
    update();
}

void DriveWidget::paintEvent( QPaintEvent* event )
{
  QColor background;
  QColor crosshair;
  if( isEnabled() )
  {
    background = Qt::white;
    crosshair = Qt::black;
  }
  else
  {
    background = Qt::lightGray;
    crosshair = Qt::darkGray;
  }

  int w = width();
  int h = height();
  int size = (( w > h ) ? h : w) - 1 - 60;
  int hpad = ( w - size ) / 2 + 30;
  int vpad = ( h - size ) / 2;

  QPainter painter( this );
  painter.setBrush( background );
  painter.setPen( crosshair );

  painter.drawRect( QRect( hpad , vpad, size, size ));

  // painter.drawLine( hpad, height() / 2, hpad + size, height() / 2 );
  painter.drawLine( hpad + size / 2, vpad, hpad + size / 2, vpad + size );
  painter.drawLine( hpad, vpad + (size / 2)*1.588, hpad + size, vpad + (size / 2)*(1.0-0.588));
  painter.drawLine( hpad, vpad + (size / 2)*(1.0-0.588), hpad + size, vpad + (size / 2)*1.588);
  // painter.drawLine( hpad + size / 2, vpad + size / 2, hpad + size, vpad);

  // painter.setPen(QPen(Qt::darkGray, 3));
  // painter.drawLine( hpad + size + 20, vpad, hpad + size + 20, vpad + size );

  // if (isEnabled() && (angular_velocity_ != 0 || linear_velocity_ != 0)) {
  //   painter.setPen(QPen(Qt::darkRed, 3));    
  //   painter.drawEllipse( hpad + size + 10, (-z_velocity_ + 1.0)/2 * size + vpad - 10, 20, 20 );    
  // } else {
  //   painter.setPen(QPen(Qt::gray, 1));
  //   painter.drawEllipse( hpad + size + 10, (-z_velocity_ + 1.0)/2 * size + vpad - 10, 20, 20 );    
  //   painter.setPen( crosshair );
  // }


  if( isEnabled() && (angular_velocity_ != 0 || linear_velocity_ != 0 ))
  {
    QPen pen;
    pen.setWidth( size/150 );
    pen.setColor( Qt::darkRed );
    pen.setCapStyle( Qt::RoundCap );
    pen.setJoinStyle( Qt::RoundJoin );
    painter.setPen( pen );

    QPointF joystick[ 2 ];
    joystick[ 0 ].setX( hpad + size / 2 );
    joystick[ 0 ].setY( vpad + size / 2 );

    float x, y;
    if (x_mouse_ > hpad + size / 2) {
      x = std::min( hpad + size, int(x_mouse_) );
      joystick[ 1 ].setX( x );
    } else {
      x = std::max( hpad, int(x_mouse_) );
      joystick[ 1 ].setX( x );
    }
    if (y_mouse_ > vpad + size / 2) {
      y = std::min( vpad + size, int(y_mouse_) );
      joystick[ 1 ].setY( y );
    } else {
      y = std::max( vpad, int(y_mouse_) );
      joystick[ 1 ].setY( y );
    }

    painter.drawPolyline( joystick, 2 );
    painter.drawEllipse( x - 10, y - 10, 20, 20 );
  }

}

void DriveWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);  // Always call the base class implementation first

    zVelocitySlider->setMinimumWidth(50); // Set the minimum width of the slider
    int newHeight = (( width() > height() ) ? height() : width()) - 1 - 60;
    zVelocitySlider->setMaximumHeight(newHeight);
}

void DriveWidget::mouseMoveEvent( QMouseEvent* event )
{
  sendVelocitiesFromMouse( event->x(), event->y(), width(), height() );
}

void DriveWidget::mousePressEvent( QMouseEvent* event )
{
  sendVelocitiesFromMouse( event->x(), event->y(), width(), height() );
}

// void DriveWidget::wheelEvent( QWheelEvent* event ) {
//   if ( event->angleDelta().y() > 0) {
//     z_velocity_ = std::min(z_velocity_ + 0.5, 1.0);
//   } else if ( event->angleDelta().y() < 0) {
//     z_velocity_ = std::max(z_velocity_ - 0.5, -1.0);
//   }
// }



void DriveWidget::leaveEvent( QEvent* event )
{
  stop();
}

void DriveWidget::mouseReleaseEvent( QMouseEvent* event )
{
  stop();
}

void DriveWidget::sendVelocitiesFromMouse( int x, int y, int width, int height )
{
  int size = (( width > height ) ? height : width );
  int hpad = ( width - size ) / 2;
  int vpad = ( height - size ) / 2;

  x_mouse_ = x;
  if ( x_mouse_ < width / 2 - size / 2 ) x_mouse_ = width / 2 - size / 2;
  else if ( x_mouse_ > width / 2 + size / 2 ) x_mouse_ = width / 2 + size / 2;
  y_mouse_ = y;
  if ( y_mouse_ < height / 2 - size / 2 ) y_mouse_ = height / 2 - size / 2;
  else if ( y_mouse_ > height / 2 + size / 2 ) y_mouse_ = height / 2 + size / 2;

  linear_velocity_ = (1.0 - float( y - vpad ) / float( size / 2 )) * linear_scale_;
  if ( linear_velocity_ < -1.0 ) linear_velocity_ = -1.0;
  else if ( linear_velocity_ > 1.0 ) linear_velocity_ = 1.0;
  if ( fabs( linear_velocity_ ) < 0.1 ) linear_velocity_ = 0;

  angular_velocity_ = ( 1.0 - float( x - hpad ) / float( size / 2 )) * angular_scale_;
  if ( angular_velocity_ < -1.0 ) angular_velocity_ = -1.0;
  else if ( angular_velocity_ > 1.0 ) angular_velocity_ = 1.0;

  mouse_pressed_ = true;
  // printf("linear_velocity: %f, angular_velocity: %f\n", linear_velocity_, angular_velocity_);

  Q_EMIT outputVelocity( linear_velocity_, angular_velocity_, mouse_pressed_, z_velocity_ );
  update();
}

void DriveWidget::stop()
{
  linear_velocity_ = 0;
  angular_velocity_ = 0;
  mouse_pressed_ = false;

  Q_EMIT outputVelocity( linear_velocity_, angular_velocity_, mouse_pressed_, z_velocity_ );
  update();
}

}
