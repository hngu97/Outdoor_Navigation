let recordingStatusTopic = new ROSLIB.Topic({
  ros : ros,
  name : 'recording/status',
  messageType : 'std_msgs/Bool'
});

let recordingStatusMsg =  new ROSLIB.Message(
  {
    data: true
  });

let toggleStatus = false;
function recordingStatusPublisher()
{
  recordingStatusMsg.data = true;
  recordingStatusTopic.publish(recordingStatusMsg);
}

let recordingStatusPubTimer = null
/*
function recordingStatusPubTimerHandle()
{
  toggleStatus = !toggleStatus
  setInterval(recordingStatusPublisher, 500);
}*/
function recordingStatusPubTimerHandle()
{
  if (recordingStatusPubTimer == null)
  {
    recordingStatusPubTimer = setInterval(recordingStatusPublisher, 500);
  }
  else
  {
    clearInterval(recordingStatusPubTimer);
    recordingStatusPubTimer = null;
  }
}
