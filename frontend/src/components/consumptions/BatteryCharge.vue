<template>
  <div>
    <v-card class="grey lighten-3">
      <v-card-title class="grey darken-1 d-flex justify-center">
        <h5 class="white--text">
          {{ $t(`consumptions.currentBatteryCharge`) }}
        </h5>
      </v-card-title>
      <v-container>
        <v-row align="center">
          <v-col sm="6">
            <v-progress-circular
              :rotate="-90"
              :size="100"
              :width="15"
              :value="this.pourcentageBatteryLeft"
              color="light-blue"
            >
              <h4 ref="batteryCharge">{{ currentBatteryCharge }} Ah</h4>
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
import { BatteryConsumption } from '@/types/batteryConsumption';
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';
import RemainingTime from '../consumptions/RemainingTime.vue';

@Component({
  components: { RemainingTime: RemainingTime },
  computed: {
    ...mapState(['batteryConsumption']),
  },
})
export default class BatteryCharge extends Vue {
  public batteryConsumption!: BatteryConsumption;
  public maximumCharge = 8;
  public trailingDecimals = 3;

  private get currentBatteryCharge() {
    return this.batteryConsumption.batteryChargeLeft.toFixed(
      this.trailingDecimals
    );
  }

  private get pourcentageBatteryLeft() {
    return (this.batteryConsumption.batteryChargeLeft / this.maximumCharge) * 100;
  }
}
</script>

<style scoped>
.v-card__title {
  padding: 0em;
}
</style>
