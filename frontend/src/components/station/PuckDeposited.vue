<template>
  <v-card class="grey lighten-3">
    <v-card-title sm="6" class="grey darken-1 d-flex justify-center">
      <h5 class="white--text">Puck deposited</h5>
    </v-card-title>
    <div class="d-flex justify-center font-weight-bold">
      <v-avatar
        ref="puckDeposited"
        size="30"
        v-for="(puck, i) in depositedPuck"
        :key="i"
        :color="puck.toString()"
        class="grey--text text--lighten-2 font-weight-bold"
      >
        {{ i + 1 }}
      </v-avatar>
    </div>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';
import { Color } from '@/types/color';
import { Step } from '@/types/step';

@Component({
  computed: {
    ...mapState(['puckColors', 'puckInGrip', 'currentStep', 'depositedPuck']),
  },
})
export default class PuckDeposited extends Vue {
  public puckColors!: Array<Color>;
  public puckInGrip!: boolean;
  public currentStep!: Step;
  public deposited: Array<Color> = [];

  get depositedPuck(): Array<Color> {
    while (this.currentStep < Step.ToFirstCornerAndReleaseFirstPuck) {
      return this.deposited;
    }
    while (
      this.currentStep == Step.ToFirstCornerAndReleaseFirstPuck &&
      this.puckInGrip
    ) {
      return this.deposited;
    }
    while (
      this.currentStep == Step.ToFirstCornerAndReleaseFirstPuck &&
      !this.puckInGrip
    ) {
      this.deposited.push(this.puckColors[0]);
      return this.deposited;
    }
    //After first release
    while (
      this.currentStep > Step.ToFirstCornerAndReleaseFirstPuck &&
      this.currentStep < Step.ToSecondCornerAndReleaseSecondPuck
    ) {
      this.deposited.push(this.puckColors[0]);
      return this.deposited;
    }
    while (
      this.currentStep == Step.ToSecondCornerAndReleaseSecondPuck &&
      this.puckInGrip
    ) {
      this.deposited.push(this.puckColors[0]);
      return this.deposited;
    }
    while (
      this.currentStep == Step.ToSecondCornerAndReleaseSecondPuck &&
      !this.puckInGrip
    ) {
      this.deposited.push(this.puckColors[0], this.puckColors[1]);
      return this.deposited;
    }
    //After second release
    while (
      this.currentStep > Step.ToSecondCornerAndReleaseSecondPuck &&
      this.currentStep < Step.ToThirdCornerAndReleaseThirdPuck
    ) {
      this.deposited.push(this.puckColors[0], this.puckColors[1]);
      return this.deposited;
    }
    while (
      this.currentStep == Step.ToThirdCornerAndReleaseThirdPuck &&
      this.puckInGrip
    ) {
      this.deposited.push(this.puckColors[0], this.puckColors[1]);
      return this.deposited;
    }
    while (
      this.currentStep == Step.ToThirdCornerAndReleaseThirdPuck &&
      !this.puckInGrip
    ) {
      return this.puckColors;
    }
    //After last release
    while (this.currentStep > Step.ToThirdCornerAndReleaseThirdPuck) {
      return this.puckColors;
    }
    return this.puckColors;
  }
}
</script>
