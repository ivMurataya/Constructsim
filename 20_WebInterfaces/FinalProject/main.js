var app = new Vue({
    el: '#app',
    // computed values
    // Camera Start
    computed: {
        ws_address: function() {
            return `${this.rosbridge_address}`
        },
    },
    // Camera END

    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        rosbridge_address: 'wss://i-0c4c9b5b62a1390b5.robotigniteacademy.com/1d6ca4aa-5f21-4ce8-a78f-56823febe493/rosbridge/',
        port: '9090',
        intervall: null,
        mapViewer: null,
        mapGridClient: null,
        // Model
        viewer: null,
        tfClient: null,
        urdfClient: null,
        // Action 
        goal: null,
        action:{
            goal :{ position : {x:0, y:0, z:0 }},
            feedback : {position : 0 , state:'idle' },
            result : { success: false },
            status : { status : 0 , text:''},
            },

// Joystick ST
        menu_title: 'Connection',
        // dragging data
        dragging: false,
        x: 'no',
        y: 'no',
        dragCircleStyle: {
            margin: '0px',
            top: '0px',
            left: '0px',
            display: 'none',
            width: '75px',
            height: '75px',
        },
        // joystick valules
        joystick: {
            vertical: 0,
            horizontal: 0,
        },
        // publisher
        pubInterval: null,   
// Joystick END
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
                this.pubInterval = setInterval(this.publish, 100)
                
                // Camera Setup 
                this.setCamera()
                this.setCamera2()

                // Model
                this.setup3DViewer()
            })

            // MAP Setup Start
            this.mapViewer = new ROS2D.Viewer({
                divID: 'map',
                width: 420,
                height: 360
            })

            // Setup the map client.
            this.mapGridClient = new ROS2D.OccupancyGridClient({
                ros: this.ros,
                rootObject: this.mapViewer.scene,
                continuous: true,
            })
            // Scale the canvas to fit to the map
            this.mapGridClient.on('change', () => {
                this.mapViewer.scaleToDimensions(this.mapGridClient.currentGrid.width, this.mapGridClient.currentGrid.height);
                this.mapViewer.shift(this.mapGridClient.currentGrid.pose.position.x, this.mapGridClient.currentGrid.pose.position.y)
            })

            // MAP Setup END


            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
                document.getElementById('divCamera').innerHTML = ''
                document.getElementById('divCamera2').innerHTML = ''
                document.getElementById('map').innerHTML = ''

                // Model
                this.unset3DViewer()
            })
        },

        disconnect: function() {
            this.ros.close()
        },



        // Joystick Start --------------------------------------------------------------------------------------------------
        publish: function() {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: this.joystick.vertical, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: this.joystick.horizontal, },
            })
            topic.publish(message)
        },

        //Send Command Funtion 
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
        //     console.log("SEND COOMMAND");
        // },
        startDrag() {
            this.dragging = true
            this.x = this.y = 0
        },
        stopDrag() {
            this.dragging = false
            this.x = this.y = 'no'
            this.dragCircleStyle.display = 'none'
            this.resetJoystickVals()
        },
        doDrag(event) {
            if (this.dragging) {
                this.x = event.offsetX
                this.y = event.offsetY
                let ref = document.getElementById('dragstartzone')
                this.dragCircleStyle.display = 'inline-block'

                let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2
                let maxTop = minTop + 200
                let top = this.y + minTop
                this.dragCircleStyle.top = `${top}px`

                let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2
                let maxLeft = minLeft + 200
                let left = this.x + minLeft
                this.dragCircleStyle.left = `${left}px`

                this.setJoystickVals()
            }
        },
        setJoystickVals() {
            this.joystick.vertical = -1 * ((this.y / 200) - 0.5)
            this.joystick.horizontal = +1 * ((this.x / 200) - 0.5)
        },
        resetJoystickVals() {
            this.joystick.vertical = 0
            this.joystick.horizontal = 0
        },
        // Joystick End --------------------------------------------------------------------------------------------------

        // Cammera Start --------------------------------------------------------------------------------------------------
        setCamera: function() {
            let without_wss = this.rosbridge_address.split('wss://')[1]
            console.log(without_wss)
            let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
            console.log(domain)
            let host = domain + '/cameras'
            let viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera',
                host: host,
                width: 400,
                height: 240,
                topic: '/camera/rgb/image_raw',
                ssl: true,
            })
        },
        setCamera2: function() {
            let without_wss = this.rosbridge_address.split('wss://')[1]
            console.log(without_wss)
            let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
            console.log(domain)
            let host = domain + '/cameras'
            let viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera2',
                host: host,
                width: 400,
                height: 240,
                topic: '/camera/depth/image_raw',
                ssl: true,
            })
        },
        // Camera End --------------------------------------------------------------------------------------------------

        // Action Start

        sendPredefinedGoal(x, y) {
            this.action.goal.position.x = x;
            this.action.goal.position.y = y;
            this.sendGoal(); // assuming this sends the current action.goal
        },

        sendGoal: function() {
            let actionClient = new ROSLIB.ActionClient({
                ros : this.ros,
                serverName : '/turtlebot2_action_service_as',
                actionName: 'course_web_dev_ros/WaypointActionAction'
            })

            this.goal = new ROSLIB.Goal({
                actionClient : actionClient,
                goalMessage : { 
                    position: this.action.goal.position
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
        },
        // Actions End
        // model Start
        setup3DViewer() {
            this.viewer = new ROS3D.Viewer({
                background: '#cccccc',
                divID: 'div3DViewer',
                width: 400,
                height: 300,
                antialias: true,
                fixedFrame: 'odom'
            })

            // Add a grid.
            this.viewer.addObject(new ROS3D.Grid({
                color:'#0181c4',
                cellSize: 0.5,
                num_cells: 20
            }))

            // Setup a client to listen to TFs.
            this.tfClient = new ROSLIB.TFClient({
                ros: this.ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0
            })

            // Setup the URDF client.
            this.urdfClient = new ROS3D.UrdfClient({
                ros: this.ros,
                param: 'robot_description',
                tfClient: this.tfClient,
                // We use "path: location.origin + location.pathname"
                // instead of "path: window.location.href" to remove query params,
                // otherwise the assets fail to load
                path: location.origin + location.pathname,
                rootObject: this.viewer.scene,
                loader: ROS3D.COLLADA_LOADER_2
            })
        },
        unset3DViewer() {
            document.getElementById('div3DViewer').innerHTML = ''
        },

        // Model End
    },
    mounted() {
    // 
    window.addEventListener('mouseup', this.stopDrag)
    // 

    this.interval = setInterval(() => {
            if (this.ros != null && this.ros.isConnected) {
                this.ros.getNodes((data) => { }, (error) => { })
            }
        }, 10000)
    },
})
