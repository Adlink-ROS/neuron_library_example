This package will create a node call uart_mraa_controller, and this node has two functions:
1. This node will subscribe to a topic 'topic' and will write the message to uart port. 
2. This node will listen to the uart port every 500ms and will publish the message from the uart to the topic 'topic1'.