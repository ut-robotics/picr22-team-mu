<script setup>
import { ref } from 'vue';
var Motorspeed = ref(0);
var Throwerspeed = ref(0);
var startButton = ref("START PROGRAM");

const collection = document.getElementsByTagName('button')

setInterval(() => {
  for (let i = 0; i < collection.length; i++) {
    if (collection[i].clicked) {
      console.log("Jou")
    }
  }
}, 50)

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
          clicked: type,
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
          speed: Throwerspeed.value
        })
      })
      break;
    case "START PROGRAM":
    case "STOP PROGRAM":
      fetch("/api", {
        method: "PUT",
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          clicked: (type == "START PROGRAM") ? "start" : "stop",
          speed: 0
        })
      })
      startButton.value = (type == "START PROGRAM") ? "STOP PROGRAM" : "START PROGRAM"
      break;
    default:
      break
  }
}

</script>

<template>
  <div class="app">
    <div class="arrows">
      <div class="start">
        <button @click="(e) => buttonClicked(e, startButton)">{{ startButton }}</button>
      </div>
      <div class="up">
        <button @click="(e) => buttonClicked(e, 'spinLeft')">&#8630;</button>
        <button @click="(e) => buttonClicked(e, 'forward')">&#8593;</button>
        <button @click="(e) => buttonClicked(e, 'spinRight')">&#8631;</button>
      </div>
      <div class="middle">
        <button @click="(e) => buttonClicked(e, 'left')">&#8592;</button>
        <button @click="(e) => buttonClicked(e, 'throw')">T</button>
        <button @click="(e) => buttonClicked(e, 'right')">&#8594;</button>
      </div>
      <div class="down">
        <button @click="(e) => buttonClicked(e, 'backward')">&#8595;</button>
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
