<template>
  <div>
    <span>{{ this.updatedTime }}</span>
    <StartButton @start="start" />
  </div>
</template>

<script lang="ts">
import { Step } from '@/types/step';
import { Component, Vue } from 'vue-property-decorator';
import { mapState, mapActions } from 'vuex';
import StartButton from './StartButton.vue';

@Component({
  components: {
    StartButton: StartButton,
  },
  methods: {
    ...mapActions(['emitSocketStartCycle']),
  },
  computed: {
    ...mapState(['cycleReady', 'cycleStarted', 'currentStep']),
  },
})
export default class Chronometer extends Vue {
  public emitSocketStartCycle!: () => void;
  public cycleReady!: boolean;
  public cycleStarted!: boolean;
  public currentStep!: Step;

  public elapsedTime: any = 0; //TODO CHANGE TYPE
  public stopwatchInterval: any = 0; //TODO CHANGE TYPE
  public prevTime: any = 0; //TODO CHANGE TYPE

  //TODO: LOGIC WITH STEP AND BOOLEANS
  public start() {
    this.emitSocketStartCycle();

    if (!this.stopwatchInterval) {
      this.stopwatchInterval = setInterval(() => {
        if (this.currentStep === Step.CycleEndedAndRedLedOn) {
          this.stop();
        } else {
          if (!this.prevTime) {
            this.prevTime = Date.now();
          }

          this.elapsedTime += Date.now() - this.prevTime;
          this.prevTime = Date.now();

          this.updatedTime;
        }
      }, 50);
    }
  }

  public stop() {
    if (this.stopwatchInterval) {
      clearInterval(this.stopwatchInterval);
      this.stopwatchInterval = null;
    }
    this.prevTime = null;
  }

  public reset() {
    this.elapsedTime = 0;
    this.updatedTime;
  }

  get updatedTime() {
    let tempTime = this.elapsedTime;
    const milliseconds = tempTime % 1000;
    tempTime = Math.floor(tempTime / 1000);
    const seconds = tempTime % 60;
    tempTime = Math.floor(tempTime / 60);
    const minutes = tempTime % 60;
    tempTime = Math.floor(tempTime / 60);
    const hours = tempTime % 60;

    return `${hours} : ${minutes} : ${seconds}.${milliseconds}`;
  }
}
</script>

<style></style>
