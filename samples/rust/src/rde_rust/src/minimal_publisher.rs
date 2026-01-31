use rclrs::{Context, Node, Publisher, RclrsError, QOS_PROFILE_DEFAULT};
use std_msgs::msg::String as StringMsg;
use std::sync::{Arc, Mutex};
use std::time::Duration;

fn main() -> Result<(), RclrsError> {
    let context = Context::new(std::env::args())?;
    let node = Node::new(&context, "minimal_publisher")?;
    
    let publisher = node.create_publisher::<StringMsg>("topic", QOS_PROFILE_DEFAULT)?;
    let publisher = Arc::new(Mutex::new(publisher));
    
    let mut count = 0u32;
    let _timer = node.create_wall_timer(Duration::from_millis(500), move || {
        let msg = StringMsg {
            data: format!("Hello from Rust! count: {}", count),
        };
        
        println!("Publishing: '{}'", msg.data);
        publisher.lock().unwrap().publish(msg).unwrap();
        count += 1;
    })?;
    
    rclrs::spin(&node)?;
    
    Ok(())
}
