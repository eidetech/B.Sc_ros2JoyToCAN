
var socket = io();
socket.on('connect', function() {
    console.log("Connected to socket.")
});


// socket.on('connect', function(socket) {
//     console.log('Connected to socket with id: ' + socket.id);
//     socket.emit('test');
// });


setInterval(function() {
    socket.emit("angVel");
}, 10);


socket.on('angVel_q1', function(info) {
    $('#angVel_q1').text(info);
});

socket.on('angVel_q2', function(info) {
    $('#angVel_q2').text(info);
});

socket.on('t', function(percent) {
    document.getElementById("progress_bar").style.width = percent + "%";
});
