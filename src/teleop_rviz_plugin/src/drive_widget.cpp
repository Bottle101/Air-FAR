#include "drive_widget.h"
#include "ros/ros.h"

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
  int hpad = ( w - size ) / 2;
  int vpad = ( h - size ) / 2;

  QPainter painter( this );
  painter.setBrush( background );
  painter.setPen( crosshair );

  painter.drawRect( QRect( hpad, vpad, size, size ));

  painter.drawLine( hpad, height() / 2, hpad + size, height() / 2 );
  painter.drawLine( width() / 2, vpad, width() / 2, vpad + size );

  // painter.setPen(QPen(Qt::darkGray, 3));
  painter.drawLine( hpad + size + 20, vpad, hpad + size + 20, vpad + size );

  if (isEnabled() && (angular_velocity_ != 0 || linear_velocity_ != 0)) {
    painter.setPen(QPen(Qt::darkRed, 3));    
    painter.drawEllipse( hpad + size + 10, (-z_velocity_ + 1.0)/2 * size + vpad - 10, 20, 20 );    
  } else {
    painter.setPen(QPen(Qt::gray, 1));
    painter.drawEllipse( hpad + size + 10, (-z_velocity_ + 1.0)/2 * size + vpad - 10, 20, 20 );    
    painter.setPen( crosshair );
  }


  if( isEnabled() && (angular_velocity_ != 0 || linear_velocity_ != 0 ))
  {
    QPen pen;
    pen.setWidth( size/150 );
    pen.setColor( Qt::darkRed );
    pen.setCapStyle( Qt::RoundCap );
    pen.setJoinStyle( Qt::RoundJoin );
    painter.setPen( pen );

    QPointF joystick[ 2 ];
    joystick[ 0 ].setX( w/2 );
    joystick[ 0 ].setY( h/2 );
    joystick[ 1 ].setX( x_mouse_ );
    joystick[ 1 ].setY( y_mouse_ );

    painter.drawPolyline( joystick, 2 );
    painter.drawEllipse( x_mouse_ - 10, y_mouse_ - 10, 20, 20 );
    // painter.drawEllipse( hpad + size + 10, (-z_velocity_ + 1.0)/2 * size + vpad - 10, 20, 20 );
  }
}

void DriveWidget::mouseMoveEvent( QMouseEvent* event )
{
  sendVelocitiesFromMouse( event->x(), event->y(), width(), height() );
}

void DriveWidget::mousePressEvent( QMouseEvent* event )
{
  sendVelocitiesFromMouse( event->x(), event->y(), width(), height() );
}

// void DriveWidget::KeyPressEvent( QKeyEvent* event ) {
//   if ( event->key() == Qt::Key_Up || event->key() == Qt::Key_W) {
//     z_velocity_ = 1.0;
//   } else if ( event->key() == Qt::Key_Down || event->key() == Qt::Key_S) {
//     z_velocity_ = -1.0;
//   } else {
//     z_velocity_ = 0.0;
//   }
//   z_velocity_ = 1.0;
//   ROS_WARN("z_velocity: %f", z_velocity_);
//   // printf("z_velocity: %f\n", z_velocity_);
// }

void DriveWidget::wheelEvent( QWheelEvent* event ) {
  if ( event->angleDelta().y() > 0) {
    z_velocity_ = std::min(z_velocity_ + 0.5, 1.0);
  } else if ( event->angleDelta().y() < 0) {
    z_velocity_ = std::max(z_velocity_ - 0.5, -1.0);
  }
  // z_velocity_ = 1.0;
  // ROS_WARN("z_velocity: %f", z_velocity_);
  // printf("z_velocity: %f\n", z_velocity_);
}



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
