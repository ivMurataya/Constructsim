let vueApp = new Vue({
    el: "#vueApp",
    data: {
        //ros connection
        ros: null,
        rosbridge_address: 'wss://i-0b179fcdab3851a5d.robotigniteacademy.com/1ed43a78-c51c-4eee-987c-0eeaac4e833e/rosbridge/',
        connected: false,
        // Page content
        menu_title: 'My menu title',
        main_title: 'Main title, from Vue!!',
        content_1: "VUE This is the left side of my web page. It occupies 33% of the total width",
        content_2: "VUE Here it goes the main content of my web page.",
    },

    methods:{
        connect: function(){
            // Define ROSBridge connection object
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })

            // Define callbacks
            this.ros.on('connection',()=>{
                this.connected = true,
                console.log('Connection to ROSBridge established!');
            })

            this.ros.on('error', (error)=>{
                console.log('Something went wrong when trying to connect');
                console.log(error);
            })

            this.ros.on('close', () => {
                this.connected = false,
                console.log('Connection to ROSBridge was closed!');
            })
        },
        disconnect: function(){
            this.ros.close()
        },
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
                    linear.x = 1;
                    console.log('forward');
                    break;
                case 'left':
                    angular.z = 0.5;
                    console.log('left');
                    break;
                case 'right':
                    angular.z = -0.5;
                    console.log('rigth');
                    break;
                case 'stop':
                    linear.x = 0;
                    angular.z = 0;
                    console.log('STOP');
                    break;
                default:
                    linear.x = 0.5;
                    angular.z = 0.3;
                    console.log('Default');
            }

            let message = new ROSLIB.Message({
                linear,
                angular,
            })
            topic.publish(message)
            
        },
    },
    mounted(){
        // Page is ready
        console.log("Page is Ready!");        
    },
})
