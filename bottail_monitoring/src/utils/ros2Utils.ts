// utils/ros.ts
import * as ROSLIB from 'roslib';

let ros: ROSLIB.Ros | null = null;
let orderTopic: ROSLIB.Topic<{data: string}> | null = null;

export const connectROS = () => {
  ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090',
  });

  ros.on('connection', () => {
    if(ros == null) {
        console.log('ROS is null');
    }else{
        console.log('ROS2 Connected');
    
        orderTopic = new ROSLIB.Topic({
          ros,
          name: '/dsr01/cocktail/command',
          messageType: 'std_msgs/String',
        });
    }
  });

  ros.on('error', (error) => {
    console.error('ROS Error', error);
  });
};

export const publishOrderTopic = (data: string) => {
  if (!orderTopic) {
    console.warn('ROS not connected yet');
    return;
  }

  orderTopic.publish({ data });
  console.log(`${data} Topic Published`)
};
