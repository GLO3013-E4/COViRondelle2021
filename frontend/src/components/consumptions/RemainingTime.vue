<template>
  <h2 :class="timeInSeconds <= minimumTimeInSecondForCompetition ? 'red--text' : 'green--text'">{{formatedTime}}</h2>
</template>

<script lang="ts">
import { RobotConsumption } from '@/types/robotConsumption';
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';

@Component({
  computed: {
    ...mapState(['robotConsumption']),
  },
})
export default class RemainingTime extends Vue {
  public robotConsumption!: RobotConsumption;

  get minimumTimeInSecondForCompetition(){
      return 10*60;
  }

  get timeInSeconds() {
      return this.robotConsumption.batteryRemainingTimeInSeconds;
  }

  get formatedTime() {
    return new Date(this.timeInSeconds * 1000).toISOString().substr(11, 8);
  } 
}
</script>
