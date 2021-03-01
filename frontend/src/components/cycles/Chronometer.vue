<template>
<div>
  
  <span>{{ this.updatedTime }}</span>
<v-btn @click="start">
    start
</v-btn>
<v-btn @click="stop">
    stop
</v-btn>
<v-btn @click="reset">
    reset
</v-btn>
</div>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';

@Component({
  components: {},
})
export default class Chronometer extends Vue {
    public elapsedTime:any = 0;
    public stopwatchInterval:any= 0;
    public prevTime:any = 0;

    start(){
        if (!this.stopwatchInterval) {
    this.stopwatchInterval = setInterval( () => {
      if (!this.prevTime) {
        this.prevTime = Date.now();
      }
      
      this.elapsedTime += Date.now() - this.prevTime;
      this.prevTime = Date.now();
      
      this.updatedTime;
    }, 50);
  }
    }

    stop(){
     if (this.stopwatchInterval) {
    clearInterval(this.stopwatchInterval);
    this.stopwatchInterval = null;
  }
  this.prevTime = null;
    }

    reset(){
      this.elapsedTime = 0;
    this.updatedTime;
   }

    get updatedTime(){
        let tempTime = this.elapsedTime;
        const milliseconds = tempTime % 1000;
        tempTime = Math.floor(tempTime / 1000);
        const seconds = tempTime % 60;
        tempTime = Math.floor(tempTime / 60);
        const minutes = tempTime % 60;
        tempTime = Math.floor(tempTime / 60);
        const hours = tempTime % 60;
  
        return hours + " : " + minutes + " : " + seconds + "." + milliseconds;
    }
}
</script>

<style></style>
