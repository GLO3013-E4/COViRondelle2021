<template>
  <div>
    <v-card class="grey lighten-3">
      <v-card-title class="grey darken-1 d-flex justify-center">
        <h5 class="white--text">Current battery charge</h5>
      </v-card-title>
      <v-container>
        <v-row align="center">
          <v-col sm="6">
            <v-progress-circular
              :rotate="-90"
              :size="100"
              :width="15"
              :value="pourcentageBatteryLeft"
              color="light-blue"
            >
              {{ currentBatteryCharge }} Ah
            </v-progress-circular>
          </v-col>
          <v-col sm="6">
            <RemainingTime />
          </v-col>
        </v-row>
      </v-container>
    </v-card>
  </div>
</template>

<script lang="ts">
import { RobotConsumption } from '@/types/robotConsumption';
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';
import RemainingTime from '../consumptions/RemainingTime.vue';

@Component({
  components: { RemainingTime: RemainingTime },
  computed: {
    ...mapState(['robotConsumption']),
  },
})
export default class BatteryCharge extends Vue {
  public robotConsumption!: RobotConsumption;
  public maximumCharge = 8;
  public trailingDecimals = 3;

  get currentBatteryCharge() {
    return this.robotConsumption.batteryChargeLeft.toFixed(
      this.trailingDecimals
    );
  }

  get pourcentageBatteryLeft() {
    return (this.robotConsumption.batteryChargeLeft / this.maximumCharge) * 100;
  }
}
</script>

<style scoped>
.v-card__title {
  padding: 0em;
}
</style>
