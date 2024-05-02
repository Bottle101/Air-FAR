#include "drive_widget.h"
#include "teleop_panel.h"

namespace teleop_rviz_plugin
{

TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
  , mouse_pressed_( false )
  , mouse_pressed_sent_( false )
{
  QVBoxLayout* layout = new QVBoxLayout;
  push_button_2_ = new QPushButton( "Resume Navigation to Goal", this );
  layout->addWidget( push_button_2_ );
  // push_button_3_ = new QPushButton( "Read", this );
  // layout->addWidget( push_button_3_ );
  // push_button_4_ = new QPushButton( "Save", this );
  // layout->addWidget( push_button_4_ );
  drive_widget_ = new DriveWidget;
  layout->addWidget( drive_widget_ );
  setLayout( layout );

  QTimer* output_timer = new QTimer( this );

  // connect( push_button_1_, SIGNAL( pressed() ), this, SLOT( pressButton1() ));
  connect( push_button_2_, SIGNAL( pressed() ), this, SLOT( pressButton2() ));
  // connect( push_button_3_, SIGNAL( pressed() ), this, SLOT( pressButton3() ));
  // connect( push_button_4_, SIGNAL( pressed() ), this, SLOT( pressButton4() ));
  // connect( check_box_1_, SIGNAL( stateChanged(int) ), this, SLOT( clickBox1(int) ));
  // connect( check_box_2_, SIGNAL( stateChanged(int) ), this, SLOT( clickBox2(int) ));
  connect( drive_widget_, SIGNAL( outputVelocity( float, float, bool, float )), this, SLOT( setVel( float, float, bool, float )));
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  output_timer->start( 100 );

  velocity_publisher_ = nh_.advertise<sensor_msgs::Joy>( "/joy", 5 );
  attemptable_publisher_ = nh_.advertise<std_msgs::Bool>( "/planning_attemptable", 5 );
  update_publisher_ = nh_.advertise<std_msgs::Bool>( "/update_visibility_graph", 5 );
  reset_publisher_ = nh_.advertise<std_msgs::Empty>( "/reset_visibility_graph", 5 );
  read_publisher_ = nh_.advertise<std_msgs::String>( "/read_file_dir", 5 );
  save_publisher_ = nh_.advertise<std_msgs::String>( "/save_file_dir", 5 );
  drive_widget_->setEnabled( true );
}

// void TeleopPanel::pressButton1()
// {
//   if ( ros::ok() && velocity_publisher_ )
//   {
//     std_msgs::Empty msg;
//     reset_publisher_.publish(msg);
//   }
// }

void TeleopPanel::pressButton2()
{
  if ( ros::ok() && velocity_publisher_ )
  {
    sensor_msgs::Joy joy;

    joy.axes.push_back(0);
    joy.axes.push_back(0);
    joy.axes.push_back(-1.0);
    joy.axes.push_back(0);
    joy.axes.push_back(1.0);
    joy.axes.push_back(1.0);
    joy.axes.push_back(0);
    joy.axes.push_back(0);

    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(1);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);

    joy.header.stamp = ros::Time::now();
    joy.header.frame_id = "teleop_panel";
    velocity_publisher_.publish( joy );
  }
}

void TeleopPanel::setVel( float lin, float ang, bool pre, float z_vel )
{
  linear_velocity_ = lin;
  angular_velocity_ = ang;
  mouse_pressed_ = pre;
  z_velocity_ = z_vel;
}

void TeleopPanel::sendVel()
{
  if( ros::ok() && velocity_publisher_ && ( mouse_pressed_ || mouse_pressed_sent_ ))
  {
    sensor_msgs::Joy joy;

    joy.axes.push_back( 0 );
    joy.axes.push_back( z_velocity_ );
    joy.axes.push_back( 1.0 );
    joy.axes.push_back( angular_velocity_ );
    joy.axes.push_back( linear_velocity_ );
    joy.axes.push_back( 1.0 );
    joy.axes.push_back( 0 );
    joy.axes.push_back( 0 );

    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 1 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );

    if (angular_velocity_/(linear_velocity_+0.0001) > 1.7 || angular_velocity_/(linear_velocity_+0.0001) < -1.7)
    {
      // if (angular_velocity_ > 0)
      // {
      //   joy.axes[0] = 1.0;
      // }
      // else if (angular_velocity_ < 0)
      // {
      //   joy.axes[0] = -1.0;
      // }
      joy.axes[0] = angular_velocity_;
      joy.axes[3] = 0.0;
      joy.axes[4] = 0.0;
      joy.axes[5] = -1.0;
    }

    joy.header.stamp = ros::Time::now();
    joy.header.frame_id = "teleop_panel";
    velocity_publisher_.publish( joy );

    mouse_pressed_sent_ = mouse_pressed_;
  }
}

void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

void TeleopPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

} 
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( teleop_rviz_plugin::TeleopPanel,rviz::Panel )
