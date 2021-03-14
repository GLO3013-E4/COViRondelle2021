<template>
  <div>
    <v-card class="grey lighten-3">
      <v-card-title class="grey darken-1 d-flex justify-center">
        <h5 class="white--text">{{ $t(`consumptions.robotConsumption`) }}</h5>
      </v-card-title>
      <v-container>
        <v-row align="center">
          <v-col sm="6">
            <h4>{{ $t(`consumptions.wheel1`) }} :</h4>
            <h4>{{ $t(`consumptions.wheel2`) }} :</h4>
          </v-col>
          <v-col sm="6">
            <h4>{{ $t(`consumptions.wheel3`) }} :</h4>
            <h4>{{ $t(`consumptions.wheel4`) }} :</h4>
          </v-col>
          <v-col sm="6">
            <h4>{{ $t(`consumptions.servoMotor`) }} :</h4>
          </v-col>
          <v-col sm="6">
            <h4>{{ $t(`consumptions.total`) }} :</h4>
          </v-col>
        </v-row>
      </v-container>
    </v-card>
  </div>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';
import RemainingTime from '../consumptions/RemainingTime.vue';

@Component({
  components: { RemainingTime: RemainingTime },
  computed: {
    ...mapState(['robotConsumption']),
  },
})
export default class BatteryCharge extends Vue {}
</script>

<style scoped>
.v-card__title {
  padding: 0em;
}
</style>
