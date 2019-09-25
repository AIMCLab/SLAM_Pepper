var self = this;

function connect(pepperIP){
  console.log("connecting to Pepper");
  document.cookie = pepperIP;

  var setupIns_ = function(){
    self.session.service("ALTextToSpeech").done(function(ins){
      self.tts = ins;
    });

    self.session.service("ALMotion").done(function(ins){
      self.alMotion =ins;
    });

    self.session.service("ALBehaviorManager").done(function(ins){
      self.alBehavior = ins;
    });

    self.session.service("ALAnimatedSpeech").done(function(ins){
      self.alAnimatedSpeech = ins;
    });

    self.session.service("ALRobotPosture").done(function(ins){
      self.alRobotPosture = ins;
    });
  }

  self.session = new QiSession(pepperIP);
  self.session.socket().on('connect', function(){
    self.session.service("ALTextToSpeech").done(function(tts){
      tts.say("Connected");
    });
    setupIns_();
    document.getElementById('connector').style.display = 'none';
    document.getElementById('connectionStatus').innerHTML = 'Connected to: '+pepperIP;
    document.getElementById('controller').style.display = 'inherit';


  }).on('disconnect', function(){
    alert("disconnected");
  });
}
function say(){
  console.log("say function initiated.");
  var text = document.getElementsByName("sayText")[0].value;
  this.tts.say(text);
}

function move(to){
  if(self.alMotion){
    console.log("move function initiated.");
    switch (to) {
      case 0:
          //move forward
          self.alMotion.moveTo(0.3, 0, 0).fail(function(err){console.log(err);});
          break;

      case 1:
          //face left
          self.alMotion.moveTo(0, 0, 0.5).fail(function(err){
            console.log(err);
          });
  				break;

      case 2:
          //move backward
          self.alMotion.moveTo(-0.3, 0, 0).fail(function(err){
            console.log(err);
          });
  				break;

      case 3:
          //face right
          self.alMotion.moveTo(0, 0, -0.5).fail(function(err){
            console.log(err);
          });
  				break;

      default:
          self.alMotion.moveTo(0, 0, 0).fail(function(err){
            console.log(err);
          });
          break;
    }
  }
}

function standGesture(){
console.log(document.cookie);
console.log("stand function initiated");

  if(self.alMotion){
    console.log("if");
    self.alRobotPosture.goToPosture("Stand", 0.8);
    // self.alBehavior.runBehavior("animations/Stand/Emotions/Positive/Laugh_1");
  }
}

function handsUp(){
  console.log("hands up initiated");
  if(self.alBehavior){
    console.log("if");
    self.alBehavior.runBehavior("animations/Stand/Gestures/ShowSky_8");
  }
}
