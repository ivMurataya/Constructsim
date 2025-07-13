var app = new Vue({
    el: '#app',
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        rosbridge_address: 'wss://i-046df00ad92e59398.robotigniteacademy.com/6e7afcd8-d55e-48c7-ba4a-b3e95c04f3dd/rosbridge/',
        // port: '9090',
        service_busy: false,
        takeoff_response: '',
        landing_response: ''
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
        TakeOff: function() {
            // service is busy
            this.service_busy = true
            this.takeoff_response = ''
            // define the service to be called
            let service = new ROSLIB.Service({
                ros: this.ros,
                name: '/hector_services/takeoff',
                serviceType: 'std_srvs/SetBool',
            })

            // define the request
            let request = new ROSLIB.ServiceRequest({
                data: true,
            })

            // define a callback
            service.callService(request, (result) => {
                this.service_busy = false
                this.takeoff_response = JSON.stringify(result)
            })
        },
        Landing: function() {
            // service is busy
            this.service_busy = true
            this.landing_response = ''
            // define the service to be called
            let service = new ROSLIB.Service({
                ros: this.ros,
                name: '/hector_services/landing',
                serviceType: 'std_srvs/SetBool',
            })

            // define the request
            let request = new ROSLIB.ServiceRequest({
                data: true,
            })

            // define a callback
            service.callService(request, (result) => {
                this.service_busy = false
                this.landing_response = JSON.stringify(result)
            })
        },
    },
})
