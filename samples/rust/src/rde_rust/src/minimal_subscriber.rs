use rclrs::{Context, Node, Subscription, RclrsError, QOS_PROFILE_DEFAULT};
use std_msgs::msg::String as StringMsg;

fn main() -> Result<(), RclrsError> {
    let context = Context::new(std::env::args())?;
    let node = Node::new(&context, "minimal_subscriber")?;
    
    let _subscription = node.create_subscription::<StringMsg, _>(
        "topic",
        QOS_PROFILE_DEFAULT,
        move |msg: StringMsg| {
            println!("I heard: '{}'", msg.data);
        },
    )?;
    
    rclrs::spin(&node)?;
    
    Ok(())
}
