<template>
<div>
  
  <span>{{ this.elapsedTime }}</span>
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
    public startTime = 0;
    public elapsedTime = 0;
    public timer!:number;

    start(){
        this.timer = setInterval(() => {
        this.elapsedTime += 1000;
      }, 1000);
    }

    stop(){
        clearInterval(this.timer);
    }

    reset(){
        this.elapsedTime = 0;
    }
}
</script>

<style></style>
