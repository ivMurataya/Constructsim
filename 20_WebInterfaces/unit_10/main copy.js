var app = new Vue({
    el: '#app',
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        rosbridge_address: 'wss://i-0433f4f1021b5eb91.robotigniteacademy.com/520d9c24-fc42-4bd3-8e92-87379ae2bb4b/rosbridge/',
        port: '9090',
        goal: null,
        action:{
            goal :{ total: 0},
            feedback : {progress : 0 },
            result : { result : 0 },
            status : { status : 0 , text : " "},
            }
    },
    // helper methods to connect to ROS
    methods: {
        connect: function() {
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false
            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
            })
        },
        disconnect: function() {
            this.ros.close()
        },

        sendGoal: function() {
            let actionClient = new ROSLIB.ActionClient({
                ros : this.ros,
                serverName : '/example_action_service_as',
                actionName: 'course_web_dev_ros/ExampleActionAction'
            })

            this.goal = new ROSLIB.Goal({
                actionClient : actionClient,
                goalMessage : { 
                    total: this.action.goal.total
                }
            })
            
            
            this.goal.on('status',(status)=>{
                this.action.status = status
            })
            this.goal.on('feedback',(feedback)=>{
                this.action.feedback = feedback
            })
            this.goal.on('result',(result)=>{
                this.action.result = result
            })           

            this.goal.send()
        },

        cancelGoal: function(){
            this.goal.cancel()
        }
    },
    mounted() {
    },
})
