<template>
  <v-card class="grey lighten-3" height="100%">
    <v-card-title sm="6" class="grey darken-1 d-flex justify-center">
      <h5 class="white--text">Puck Deposit</h5>
    </v-card-title>
     <v-container height="100%">
      <v-row align="center">
        <v-col sm="12">
          <div ref="corner" class="d-flex justify-center font-weight-bold">
      <v-avatar
        ref="puckDeposited"
        size="30"
        v-for="(puck, i) in depositedPucks"
        :key="i"
        :color="puck.toString()"
        class="grey--text text--lighten-2 font-weight-bold"
      >
        {{ i + 1 }}
      </v-avatar>
    </div>
        </v-col>
      </v-row>
     </v-container>
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
export default class PuckDeposit extends Vue {
  public puckColors!: Array<Color>;
  public puckInGrip!: boolean;
  public currentStep!: Step;
  public deposited: Array<Color> = [];

  get noPuckYet(): boolean {
    return (
      this.currentStep < Step.ToFirstCornerAndReleaseFirstPuck ||
      (this.currentStep == Step.ToFirstCornerAndReleaseFirstPuck &&
        this.puckInGrip)
    );
  }

  get firstPuckDeposited(): boolean {
    return (
      (this.currentStep == Step.ToFirstCornerAndReleaseFirstPuck &&
        !this.puckInGrip) ||
      (this.currentStep > Step.ToFirstCornerAndReleaseFirstPuck &&
        this.currentStep < Step.ToSecondCornerAndReleaseSecondPuck) ||
      (this.currentStep == Step.ToSecondCornerAndReleaseSecondPuck &&
        this.puckInGrip)
    );
  }

  get secondPuckDeposited(): boolean {
    return (
      (this.currentStep == Step.ToSecondCornerAndReleaseSecondPuck &&
        !this.puckInGrip) ||
      (this.currentStep > Step.ToSecondCornerAndReleaseSecondPuck &&
        this.currentStep < Step.ToThirdCornerAndReleaseThirdPuck) ||
      (this.currentStep == Step.ToThirdCornerAndReleaseThirdPuck &&
        this.puckInGrip)
    );
  }

  get thirdPuckDeposited(): boolean {
    return (
      (this.currentStep == Step.ToThirdCornerAndReleaseThirdPuck &&
        !this.puckInGrip) ||
      this.currentStep > Step.ToThirdCornerAndReleaseThirdPuck
    );
  }

  get depositedPucks(): Array<Color> {
    if (this.noPuckYet) {
      return this.deposited;
    } else if (this.firstPuckDeposited) {
      this.deposited.push(this.puckColors[0]);
      return this.deposited;
    } else if (this.secondPuckDeposited) {
      this.deposited.push(this.puckColors[0], this.puckColors[1]);
      return this.deposited;
    } else if (this.thirdPuckDeposited) {
      return this.puckColors;
    }
    return this.puckColors;
  }
}
</script>
