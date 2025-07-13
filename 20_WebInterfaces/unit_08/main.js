var app = new Vue({
    el: '#app',
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        rosbridge_address: 'wss://i-0d47fc4b48d5438bb.robotigniteacademy.com/028c0817-38a8-4ae7-a756-4b50293d34d9/rosbridge/',
        port: '9090',
        service_busy: false,
        param_valX: 0,
        // param_val: 0,
        param_valZ: 0,
        param_read_valX: null,
        param_read_valZ: null,
 
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

        set_zero: function() {
            this.set_param('/web_param/angular_z',0)
            this.set_param('/web_param/linear_x',0)

        },

        set_param: function(p_name, p_value) {
            // set as busy
            service_busy = true

            let web_param = new ROSLIB.Param({
                ros: this.ros,
                name: p_name
            })

            web_param.set(p_value)

            // set as not busy
            service_busy = false
        },
        read_param: function(p_name, callback) {
            // set as busy
            service_busy = true

            let web_param = new ROSLIB.Param({
                ros: this.ros,
                name: p_name
            })

            // web_param.get((value) => {
            //     // set as not busy
            //     service_busy = false
            //     this.param_read_valX = value
            // }, (err) => {
            //     // set as not busy
            //     service_busy = false
            // })
            
            web_param.get((value) => {
                service_busy = false;
                console.log('Param', p_name, 'value:', value);
                if (callback) callback(value);
            }, (err) => {
                service_busy = false;
                console.error('Error reading param:', p_name, err);
                if (callback) callback(null, err);
            })


        },
    },
    mounted() {
    },
})
