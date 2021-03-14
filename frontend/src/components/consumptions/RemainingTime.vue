<template>
  <h2 class="light-blue--text" ref="time">
    {{ formatedTime }}
  </h2>
</template>

<script lang="ts">
import { BatteryConsumption } from '@/types/batteryConsumption';
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';

@Component({
  computed: {
    ...mapState(['batteryConsumption']),
  },
})
export default class RemainingTime extends Vue {
  public batteryConsumption!: BatteryConsumption;

  private get timeInSeconds() {
    return this.batteryConsumption.batteryRemainingTimeInSeconds;
  }

  private get formatedTime() {
    return new Date(this.timeInSeconds * 1000).toISOString().substr(11, 8);
  }
}
</script>
