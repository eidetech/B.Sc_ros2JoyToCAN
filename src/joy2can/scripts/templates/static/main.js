
var socket = io();
socket.on('connect', function() {
    console.log("Connected to socket.")
});

setInterval(function() {
    socket.emit("client_data");
}, 10);

socket.on('percent', function(percent) {
    document.getElementById("progress_bar").style.width = percent + "%";
    $('#percentage').text(Math.round(percent)+ "%");
});

socket.on('pitch', function(info) {
    $('#pitch').text(info);
});

socket.on('angVel_q2', function(info) {
    $('#angVel_q2').text(info);
});