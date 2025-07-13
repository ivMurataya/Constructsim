let vueApp = new Vue({
    el: "#vueApp",
    data: {
        // ros connection
        ros: null,
        rosbridge_address: 'wss://i-07a37b6c23fbc4c9d.robotigniteacademy.com/4564146b-a8dd-4436-8f71-98dba2d7a4d8/rosbridge/',
        connected: false,
        // subscriber data
        position: { x: 0, y: 0, z: 0, },
        // fence mode
        fenceMode: false,
        insideFence: false,
        // page content
        menu_title: 'Connection',
        main_title: 'Robot Movement',
        logs: [],  // <-- Add this line
    },
    methods: {
        connect: function() {
            // define ROSBridge connection object
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })

            // define callbacks
            this.ros.on('connection', () => {
                //this.connected = true
                
                //Camera Settings
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false
                this.setCamera()
                this.setDepth()

                // Connection established
                console.log('Connection to ROSBridge established!')
                let topic = new ROSLIB.Topic({
                    ros: this.ros,
                    name: '/odometry/filtered',
                    messageType: 'nav_msgs/Odometry'
                })
                topic.subscribe((message) => {
                    this.position = message.pose.pose.position
                    // console.log(`fence mode is ${this.fenceMode}`)
                    // if (this.fenceMode) {
                    //     this.stayOnTheFence(message.pose.pose.position)
                    // }
                })
            })
            this.ros.on('error', (error) => {
                console.log('Something went wrong when trying to connect')
                console.log(error)
            })
            this.ros.on('close', () => {
                //this.connected = false
                //console.log('Connection to ROSBridge was closed!')

                // Add camera close settings
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
                document.getElementById('divCamera').innerHTML = ''
                document.getElementById('divDepth').innerHTML = ''


            })
        },
        disconnect: function() {
            this.ros.close()
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
        //     console.log("button right");
        // },
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
        // switchFenceMode: function() {
        //     this.fenceMode = !this.fenceMode
        // },
        // stayOnTheFence: function(position) {
        //     let topicToPublish = new ROSLIB.Topic({
        //         ros: this.ros,
        //         name: '/cmd_vel',
        //         messageType: 'geometry_msgs/Twist'
        //     })
        //     if (position.x > -5 && position.x < 5 && position.y > -5 && position.y < 5) {
        //         // we are inside the fence!
        //         this.insideFence = true
        //         let message = new ROSLIB.Message({
        //             linear: { x: 0.5, y: 0, z: 0, },
        //             angular: { x: 0, y: 0, z: 0, },
        //         })
        //         topicToPublish.publish(message)
        //     } else {
        //         // we are outside the fence!
        //         this.insideFence = false
        //         let message = new ROSLIB.Message({
        //             linear: { x: 0.5, y: 0, z: 0, },
        //             angular: { x: 0, y: 0, z: 0.5, },
        //         })
        //         topicToPublish.publish(message)
        //     }
        // },

        mounted() {
            // page is ready
            console.log('page is ready!')
        },
        setCamera: function() {
            let without_wss = this.rosbridge_address.split('wss://')[1]
            console.log(without_wss)
            let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
            console.log(domain)
            let host = domain + '/cameras'
            console.log(host);
            
            let viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera',
                host: host,
                width: 320,
                height: 240,
                topic: '/camera/rgb/image_raw',
                ssl: true,
            })
        },


        setDepth: function() {
            let without_wss = this.rosbridge_address.split('wss://')[1]
            console.log(without_wss)
            let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
            console.log(domain)
            let host = domain + '/cameras'
            console.log(host);
            
            let viewer = new MJPEGCANVAS.Viewer({
                divID: 'divDepth',
                host: host,
                width: 320,
                height: 240,
                topic: '/camera/depth/image_raw',
                ssl: true,
            })
        },
    },
        

})