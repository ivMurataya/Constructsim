let vueApp = new Vue({
    el: "#vueApp",
    data: {
        // ros connection
        ros: null,
        rosbridge_address: 'wss://i-0c94b7623a22491cf.robotigniteacademy.com/9e1377fb-6b9e-4478-a211-0486e9223089/rosbridge/',
        connected: false,
        // page content
        menu_title: 'Connection',
        main_title: 'Main title, from Vue!!',
        // Position
        position : {x:0 , y:0, z:0},
        // Fence Options
        fence_mode : false,
        direction: false,


    },
    methods: {
        connect: function() {
            // define ROSBridge connection object
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })

            // define callbacks
            this.ros.on('connection', () => {
                this.connected = true
                console.log('Connection to ROSBridge established!')
                let topic = new ROSLIB.Topic({
                    ros: this.ros,
                    name: '/odom',
                    messageType: 'nav_msgs/Odometry'
                })
                topic.subscribe((message) => {
                    // console.log(message)
                    this.position = message.pose.pose.position
                    if (this.fence_mode){
                        let d = (this.direction) ?  'right' :  'left';    
                        console.log(d);

                        if (this.position.x > 2.0  || this.position.x < -2.0 || this.position.y > 2.0  || this.position.y < -2.0 ){
                            
                            this.sendCommand(d);
                            // console.log("Out of Range. X: ", this.position.x , " Y: ", this.position.y);
                        }
                        else {
                            // console.log("In Range" ,this.position.x);
                            this.sendCommand('forward');

                        }
                    }
                
                })
            })
            this.ros.on('error', (error) => {
                console.log('Something went wrong when trying to connect')
                console.log(error)
            })
            this.ros.on('close', () => {
                this.connected = false
                console.log('Connection to ROSBridge was closed!')
            })
        },

        disconnect: function() {
            this.ros.close()
        },

        fence: function () {
            this.direction = !this.direction
            this.fence_mode = !this.fence_mode
            
            // console.log(this.fence_mode);
        },

        // sendCommand: function() {
        //     let topic = new ROSLIB.Topic({
        //         ros: this.ros,
        //         name: '/cmd_vel',
        //         messageType: 'geometry_msgs/Twist'
        //     })
        //     let message = new ROSLIB.Message({
        //         linear: { x: 1, y: 0, z: 0, },
        //         angular: { x: 0, y: 0, z: 0.5, },
        //     })
        //     topic.publish(message)
        // },
        // turnRight: function() {
        //     let topic = new ROSLIB.Topic({
        //         ros: this.ros,
        //         name: '/cmd_vel',
        //         messageType: 'geometry_msgs/Twist'
        //     })
        //     let message = new ROSLIB.Message({
        //         linear: { x: 1, y: 0, z: 0, },
        //         angular: { x: 0, y: 0, z: -0.5, },
        //     })
        //     topic.publish(message)
        // },

        sendCommand: function(direction) {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })

            let linear = { x: 0, y: 0, z: 0 };
            let angular = { x: 0, y: 0, z: 0 };

            switch (direction) {
                case 'forward':
                    linear.x = 0.2;
                    // console.log('forward');
                    break;
                case 'left':
                    linear.x = 0.35
                    angular.z = 0.5;
                    // console.log('left');
                    break;
                case 'right':
                    linear.x = 0.35
                    angular.z = -0.5;
                    // console.log('rigth');
                    break;
                case 'back':
                    linear.x = -0.5
                    // console.log('back');
                    break;
                case 'stop':
                    linear.x = 0;
                    angular.z = 0;
                    // console.log('STOP');
                    break;
                // default:
                //     linear.x = 0.0;
                //     angular.z = 0.0;
                //     console.log('Default');
            }

            let message = new ROSLIB.Message({
                linear,
                angular,
            })
            topic.publish(message)
            
        },
        // stop: function() {
        //     let topic = new ROSLIB.Topic({
        //         ros: this.ros,
        //         name: '/cmd_vel',
        //         messageType: 'geometry_msgs/Twist'
        //     })
        //     let message = new ROSLIB.Message({
        //         linear: { x: 0, y: 0, z: 0, },
        //         angular: { x: 0, y: 0, z: 0, },
        //     })
        //     topic.publish(message)
        // },
    },
    mounted() {
        // page is ready
        console.log('page is ready!')
    },
})