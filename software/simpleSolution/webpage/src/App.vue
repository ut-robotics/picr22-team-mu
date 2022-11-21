<script setup>
import { ref } from 'vue';
var Motorspeed = ref(10);
var Throwerspeed = ref(1000);

var pressedButton = ref(0);
var throwerEnabled = ref(false)

function buttonClicked(e, type) {

  switch (type) {
    case "forward":
    case "backward":
    case "left":
    case "spinLeft":
    case "spinRight":
    case "right":
      fetch("/api", {
        method: "PUT",
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          clicked: type == pressedButton.value ? 'stop' : type,
          speed: Motorspeed.value
        })
      })
      break;
    case "throw":
      fetch("/api", {
        method: "PUT",
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          clicked: type,
          speed: throwerEnabled.value ? 0 : Throwerspeed.value
        })
      })
      break;
    case "start":
      fetch("/api", {
        method: "PUT",
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          clicked: type == pressedButton.value ? 'stop' : type,
          speed: 0
        })
      })
      throwerEnabled.value = false
      break;
    default:
      break
  }
  if (type == "throw") {
    throwerEnabled.value = !throwerEnabled.value
    if (pressedButton.value == 'start') {
      pressedButton.value = 0
      fetch("/api", {
        method: "PUT",
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          clicked: 'stop',
          speed: 0
        })
      })
    }
  } else if (pressedButton.value == type)
    pressedButton.value = 0
  else
    pressedButton.value = type
}

</script>

<template>
  <div class="app">
    <div class="arrows">
      <div class="start">
        <button :class="{'selectedButton': pressedButton == 'start'}" @click="(e) => buttonClicked(e, 'start')">START PROGRAM</button>
      </div>
      <div class="up">
        <button :class="{'selectedButton': pressedButton == 'spinLeft'}" @click="(e) => buttonClicked(e, 'spinLeft')">&#8630;</button>
        <button :class="{'selectedButton': pressedButton == 'forward'}" @click="(e) => buttonClicked(e, 'forward')">&#8593;</button>
        <button :class="{'selectedButton': pressedButton == 'spinRight'}" @click="(e) => buttonClicked(e, 'spinRight')">&#8631;</button>
      </div>
      <div class="middle">
        <button :class="{'selectedButton': pressedButton == 'left'}" @click="(e) => buttonClicked(e, 'left')">&#8592;</button>
        <button :class="{'selectedButton': throwerEnabled}" @click="(e) => buttonClicked(e, 'throw')">T</button>
        <button :class="{'selectedButton': pressedButton == 'right'}" @click="(e) => buttonClicked(e, 'right')">&#8594;</button>
      </div>
      <div class="down">
        <button :class="{'selectedButton': pressedButton == 'backward'}" @click="(e) => buttonClicked(e, 'backward')">&#8595;</button>
      </div>
      <div class="speed">
        <input
          type="range"
          min="0"
          max="2000"
          step="10"
          v-model="Throwerspeed"
        />
        <p>Thrower speed: {{ Throwerspeed }}</p>
      </div>
      <div class="speed">
        <input type="range" min="0" max="50" step="1" v-model="Motorspeed" />
        <p>Motor speed: {{ Motorspeed }}</p>
      </div>
    </div>
  </div>
</template>


<style scoped>
.app {
  display: flex;
  align-items: center;
  justify-content: center;
  flex-direction: column;
  height: 100%;
}
.arrows {
  display: flex;
  flex-direction: column;
  width: 100%;
  align-items: center;
}

.selectedButton {
  background-color: red;
}

.up,
.middle,
.down {
  margin: 10px;
  width: 100%;
  max-width: 350px;
  display: flex;
}

.up,
.middle {
  justify-content: space-between;
}
.down {
  justify-content: center;
}

.speed {
  width: 100%;
  max-width: 300px;
}
input[type="range"] {
  width: 100%;
}
button {
  font-size: 75px;
  line-height: 60px;
  width: 100px;
  height: 100px;
  padding: 0;
  margin: 0;
}
.start button {
  font-size: 20px;
  height: 50px;
  width: 200px;
  line-height: 50px;
  margin-bottom: 50px;
}
</style>
