<template>
  <div>
    <v-card class="d-flex justify-center" ref="time"
      ><h2>{{ this.updatedTime }}</h2></v-card
    >
    <StartButton ref="button" @start="start" />
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
    ...mapState(['cycleReady', 'currentStep']),
  },
})
export default class Chronometer extends Vue {
  public emitSocketStartCycle!: () => void;
  public cycleReady!: boolean;
  public currentStep!: Step;

  public elapsedTime = 0;
  public interval: number | null = 0;
  public prevTime: number | null = 0;

  public start() {
    if (
      this.cycleReady == true ||
      this.currentStep == Step.CycleEndedAndRedLedOn
    ) {
      this.emitSocketStartCycle();

      if (!this.interval) {
        this.interval = setInterval(() => {
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
  }

  public stop() {
    if (this.interval) {
      clearInterval(this.interval);
      this.interval = null;
    }
    this.prevTime = null;
  }

  get updatedTime() {
    let tempTime = this.elapsedTime;
    const milliseconds = tempTime % 1000;
    tempTime = Math.floor(tempTime / 1000);
    const seconds = tempTime % 60;
    tempTime = Math.floor(tempTime / 60);
    const minutes = tempTime % 60;

    return `${minutes} : ${seconds}.${milliseconds}`;
  }
}
</script>

<style></style>
