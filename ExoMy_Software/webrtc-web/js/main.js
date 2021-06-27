'use strict';

var isChannelReady = false;
var isInitiator = false;
var isStarted = false;
var localStream = null;
var pc;
var remoteStream = null;
var turnReady;

var pcConfig = {
  'iceServers': [{
    'urls': 'stun:stun.l.google.com:19302'
  }]
};

// Set up audio and video regardless of what devices are present.
var sdpConstraints = {
  offerToReceiveAudio: true,
  offerToReceiveVideo: true
};

/////////////////////////////////////////////
var room = 'foo';
// Could prompt for room name:
// room = prompt('Enter room name:');

var socket = io.connect();

if (room !== '') {
  socket.emit('create or join', room);
  console.log('Attempted to create or  join room', room);
  // ReneB: set isInitiator right at the start.
  isInitiator = true;
}

socket.on('created', function (room) {
  isInitiator = true;
});

socket.on('full', function (room) {
  console.log('Room ' + room + ' is full');
});

socket.on('join', function (room) {
  console.log('Another peer made a request to join room ' + room);
  console.log('This peer is the initiator of room ' + room + '!');
  isChannelReady = true;
  // ReneB: as soon as the other side does a join request (which happens after a load or reload of the other side's page), he is the initiator, so make isInitiator fase.
  // If the other side is initially the initiator and does nothing, then a page refresh at this side will set the other side's isInitiator to false (see handleRemoteHangup()) and this side will continue as the initiator.
  isInitiator = false;
});

socket.on('joined', function (room) {
  console.log('joined: ' + room);
  isChannelReady = true;
});

socket.on('log', function (array) {
  console.log.apply(console, array);
});

////////////////////////////////////////////////

function sendMessage(message) {
  console.log('Client sending message: ', message);
  socket.emit('message', message);
}

// This client receives a message
socket.on('message', function (message) {
  console.log('Client received message:', message);
  if (message === 'got user media') {
    //maybeStart();
  } else if (message.type === 'offer') {
    if (!isInitiator && !isStarted) {
      maybeStart();
    }
    pc.setRemoteDescription(new RTCSessionDescription(message));
    doAnswer();
  } else if (message.type === 'answer' && isStarted) {
    pc.setRemoteDescription(new RTCSessionDescription(message));
  } else if (message.type === 'candidate' && isStarted) {
    var candidate = new RTCIceCandidate({
      sdpMLineIndex: message.label,
      candidate: message.candidate
    });
    pc.addIceCandidate(candidate);
  } else if (message === 'bye' && isStarted) {
    handleRemoteHangup();
  } else if (message == 'toggleTwoway') {
    toggleTwoway();
  } else if (message == 'ready') {
    readyState = true;
  } else if (message == 'reload') {
    location.reload();
  }
});

////////////////////////////////////////////////////

var localVideo = document.querySelector('#localVideo');
var remoteVideo = document.querySelector('#remoteVideo');

var twoWayOn = false;
var constraintsNoAudio = {
  "audio": false,
  "video": {
    "width": {
      "ideal": "640"
    },
    "height": {
      "ideal": "480"
    },
    "frameRate": {
      "ideal": "20"
    }
  }
};

var constraintsWithAudio = {
  "audio": true,
  "video": {
    "width": {
      "ideal": "640"
    },
    "height": {
      "ideal": "480"
    },
    "frameRate": {
      "ideal": "20"
    }
  }
};

if (location.hostname == 'localhost') {
  navigator.mediaDevices.getUserMedia(constraintsNoAudio)
    .then(gotStream)
    .catch(function (e) {
      alert('getUserMedia() error: ' + e.name);
    });
}

function gotStream(stream) {
  console.log('Adding local stream.');
  localStream = stream;
  //localVideo.srcObject = stream;
  sendMessage('got user media');
  if (isInitiator) {
    maybeStart();
  }
}

if (location.hostname !== 'localhost') {
  requestTurn(
    'https://computeengineondemand.appspot.com/turn?username=41784574&key=4080218913'
  );
}

var videoTrackSender;
// ReneB: maybeStart: create peer connection, add local tracks and if initiator execute doCall: create offer and send message. setRemoteDescription is called right after maybeStart.
function maybeStart() {
  console.log('>>>>>>> maybeStart() ', isStarted, localStream, isChannelReady);
  if (!isStarted && isChannelReady) {
    console.log('>>>>>> creating peer connection');
    createPeerConnection();
    if (localStream != null) {
      localStream.getTracks().forEach(track => videoTrackSender = pc.addTrack(track, localStream));
    }
    isStarted = true;
    console.log('isInitiator', isInitiator);
    if (isInitiator) {
      doCall();
    }
  }
}

window.onbeforeunload = function () {
  sendMessage('bye');
};

/////////////////////////////////////////////////////////

function createPeerConnection() {
  try {
    pc = new RTCPeerConnection(null);
    pc.onicecandidate = handleIceCandidate;
    pc.ontrack = gotRemoteStream;
    console.log('Created RTCPeerConnnection');
  } catch (e) {
    console.log('Failed to create PeerConnection, exception: ' + e.message);
    alert('Cannot create RTCPeerConnection object.');
    return;
  }
}

function handleIceCandidate(event) {
  console.log('icecandidate event: ', event);
  if (event.candidate) {
    sendMessage({
      type: 'candidate',
      label: event.candidate.sdpMLineIndex,
      id: event.candidate.sdpMid,
      candidate: event.candidate.candidate
    });
  } else {
    console.log('End of candidates.');
  }
}

function handleCreateOfferError(event) {
  console.log('createOffer() error: ', event);
}

function doCall() {
  console.log('Sending offer to peer');
  pc.createOffer(setLocalAndSendMessage, handleCreateOfferError);
}
// ReneB: doAnswer: create answer and send message. setRemoteDescription is called just before doAnswer.
function doAnswer() {
  console.log('Sending answer to peer.');
  pc.createAnswer().then(
    setLocalAndSendMessage,
    onCreateSessionDescriptionError
  );
}

function setLocalAndSendMessage(sessionDescription) {
  pc.setLocalDescription(sessionDescription);
  console.log('setLocalAndSendMessage sending message', sessionDescription);
  sendMessage(sessionDescription);
}

function onCreateSessionDescriptionError(error) {
  trace('Failed to create session description: ' + error.toString());
}

function requestTurn(turnURL) {
  var turnExists = false;
  for (var i in pcConfig.iceServers) {
    if (pcConfig.iceServers[i].urls.substr(0, 5) === 'turn:') {
      turnExists = true;
      turnReady = true;
      break;
    }
  }
  if (!turnExists) {
    console.log('Getting TURN server from ', turnURL);
    // No TURN server. Get one from computeengineondemand.appspot.com:
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function () {
      if (xhr.readyState === 4 && xhr.status === 200) {
        var turnServer = JSON.parse(xhr.responseText);
        console.log('Got TURN server: ', turnServer);
        pcConfig.iceServers.push({
          'urls': 'turn:' + turnServer.username + '@' + turnServer.turn,
          'credential': turnServer.password
        });
        turnReady = true;
      }
    };
    xhr.open('GET', turnURL, true);
    xhr.send();
  }
}

var remoteStream;
function gotRemoteStream(e) {
  console.log('gotRemoteStream', e.track, e.streams[0]);
  // ReneB: Only renew video when there is a new stream. When only audio is added, video does not have to be renewed.
  if (remoteVideo.srcObject !== e.streams[0]) {
    remoteVideo.srcObject = e.streams[0];
    remoteStream = e.streams[0];
  }
}


function hangup() {
  console.log('Hanging up.');
  stop();
  sendMessage('bye');
}

function handleRemoteHangup() {
  console.log('Session terminated.');
  stop();
  isInitiator = false;
}

function stop() {
  if (twoWayOn == true) {
    toggleTwoway();
  }
  isStarted = false;
  pc.close();
  pc = null;
  localStream = null;
}

// Define and add behavior to buttons.

// Define action buttons.
const callButton = document.getElementById('callButton');
const hangupButton = document.getElementById('hangupButton');
const twowayButton = document.getElementById('twowayButton');
const statsButton = document.getElementById('statsButton');

// Set up initial action buttons status: disable call and hangup.
callButton.disabled = false;
hangupButton.disabled = true;
twowayButton.disabled = false;
twowayButton.style.background = 'transparent';
statsButton.disabled = false;

// Add click event handlers for buttons.
callButton.addEventListener('click', callAction);
hangupButton.addEventListener('click', hangupAction);
twowayButton.addEventListener('click', twowayAction);
statsButton.addEventListener('click', statsAction);

// ReneB: Handles call button action.
// ReneB: Currenty this initiates a page reload which renew the connection.
function callAction() {
  callButton.disabled = false;
  hangupButton.disabled = false;
  callButton.style.background = 'green';
  sendMessage('reload');
}

// ReneB: Handles hangup action.
function hangupAction() {
  callButton.disabled = false;
  hangupButton.disabled = true;
  callButton.style.background = 'transparent';
  hangup();
}

// ReneB: Toggle 2-way communication.
// To prevent race contitions the 2-way communication is first set on the remote side (where the 2-way button is pressed) and then on the Exomy.
// When the Exomy is finished it sets the readyState to true by sending a message back to the remote side.
// This indicates that the switch on both sides is completed and only then the toggleAudio function can be called again.
var readyState = true;
function twowayAction() {
  if (isStarted == true && readyState == true) {
    toggleTwoway();
  }
}

// ReneB: Shows statistics on request.
function statsAction() {
  DisplayStats();
}

// ReneB: The toggleTwoway function toggles between simplex mode and full duplex mode.
// In simplex mode only video is sent from the Exomy to the remote side.
// In full duplex mode video and audio is sent both ways.
var audioTrackSender;
function toggleTwoway() {
  readyState = false;
  twowayButton.disabled = true;
  if (twoWayOn == false) { // ReneB: Switch to full duplex mode.
    twoWayOn = true;
    if (location.hostname == 'localhost') {
      navigator.mediaDevices
        .getUserMedia({ audio: true })
        .then(stream => {
          const audioTracks = stream.getAudioTracks();
          if (audioTracks.length > 0) {
            console.log(`Using audio device: ${audioTracks[0].label}`);
            audioTrackSender = pc.addTrack(audioTracks[0], localStream);
          }
        })
        // ReneB: Use arrow function otherwise code is executed immediately.
        .then(() => doCall())
        .then(() => localVideo.srcObject = localStream)
        .then(() => sendMessage('ready'))
        .then(() => twowayButton.style.background = 'green')
        .then(() => twowayButton.disabled = false);
      // ReneB: resize video to accomodate full duplex mode.
      remoteVideo.style.width = '60%';
      localVideo.style.width = '20%';
    }
    else {
      navigator.mediaDevices
        .getUserMedia(constraintsWithAudio)
        .then(stream => {
          localStream = stream;
          const audioTracks = stream.getAudioTracks();
          const videoTracks = stream.getVideoTracks();
          if (audioTracks.length > 0) {
            console.log(`Using audio device: ${audioTracks[0].label}`);
            audioTrackSender = pc.addTrack(audioTracks[0], localStream);
          }
          if (videoTracks.length > 0) {
            console.log(`Using video device: ${videoTracks[0].label}`);

            videoTrackSender = pc.addTrack(videoTracks[0], localStream);
          }
        })
        // ReneB: Use arrow function otherwise code is executed immediately.
        .then(() => doCall())
        .then(() => localVideo.srcObject = localStream)
        .then(() => sendMessage('toggleTwoway'))  // This remote side has switched on 2-way communication. Now send message to the Emomy to also switch on 2-way communication.
        .then(() => twowayButton.style.background = 'green')
        .then(() => twowayButton.disabled = false);
      // ReneB: resize video to accomodate full duplex mode.
      remoteVideo.style.width = '60%';
      localVideo.style.width = '20%';
    }
  }
  else { // ReneB: Switch to simplex mode.
    twoWayOn = false;
    console.log(`going to remove audio: ${audioTrackSender}`);
    pc.removeTrack(audioTrackSender);
    if (location.hostname == 'localhost') {
      remoteVideo.srcObject = null;
      sendMessage('ready');
    }
    else {
      console.log(`going to remove video: ${videoTrackSender}`);
      pc.removeTrack(videoTrackSender);
      // ReneB: resize video to accomodate simplex mode.
      remoteVideo.style.width = '80%';
      localVideo.style.width = '0%';
      sendMessage('toggleTwoway'); // This remote side has switched off 2-way communication. Now send message to the Emomy to also switch off 2-way communication.
    }
    twowayButton.style.background = 'transparent';
    twowayButton.disabled = false;
    localVideo.srcObject = null;
  }
}

async function DisplayStats() {
  if (localStream != null && pc != null) {
    var localVideoWidth = localStream.getVideoTracks()[0].getSettings().width;
    var localVideoHeight = localStream.getVideoTracks()[0].getSettings().height;
    var localVideoFramerate = localStream.getVideoTracks()[0].getSettings().frameRate;
    // ReneB: Get current local video codec.
    var localVideoCodec;
    var stats = await pc.getStats(null);
    stats.forEach(stat => {
      if (!(stat.type === 'outbound-rtp' && stat.kind === 'video')) {
        return;
      }
      localVideoCodec = stats.get(stat.codecId);
    });
    document.getElementById("localVideoStats").innerHTML = `<strong>Local video stats:</strong> ${localVideoWidth}x${localVideoHeight}, ${localVideoFramerate.toFixed(1)} fps, ` + localVideoCodec.mimeType;
  }

  if (remoteStream != null && pc != null) {
    var remoteVideoWidth = remoteStream.getVideoTracks()[0].getSettings().width;
    var remoteVideoHeight = remoteStream.getVideoTracks()[0].getSettings().height;
    var remoteVideoFramerate = remoteStream.getVideoTracks()[0].getSettings().frameRate;
    // ReneB: Get current remote video codec.
    var remoteVideoCodec;
    stats = await pc.getStats(null);
    stats.forEach(stat => {
      if (!(stat.type === 'inbound-rtp' && stat.kind === 'video')) {
        return;
      }
      remoteVideoCodec = stats.get(stat.codecId);
    });
    document.getElementById("remoteVideoStats").innerHTML = `<strong>Remote video stats:</strong> ${remoteVideoWidth}x${remoteVideoHeight}, ${remoteVideoFramerate.toFixed(1)} fps, ` + remoteVideoCodec.mimeType;
  }
}