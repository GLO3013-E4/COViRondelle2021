<template>
  <v-card class="lighten2" height="100%">
    <v-card-title sm="6" class="lighten1 d-flex justify-center">
      <h5 class="white--text">{{ $t('station.puckDeposit') }}</h5>
    </v-card-title>
    <v-container height="100%">
      <v-row align="center">
        <v-col sm="6">
          <div ref="corner" class="d-flex justify-center font-weight-bold">
            <v-avatar
              ref="puckDeposited"
              size="30"
              v-for="(puck, i) in depositedPucks"
              :key="i"
              :color="puck.toString()"
              class="lighten3--text font-weight-bold"
            >
              {{ i + 1 }}
            </v-avatar>
          </div>
        </v-col>
        <v-col sm="3">
          <v-btn color="primary" @click="testChangeStep">+step</v-btn>
        </v-col>
        <v-col sm="3">
          <v-btn color="accent" @click="testChangeGrip">changeGrip</v-btn>
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapMutations, mapState } from 'vuex';
import { Color } from '@/types/color';
import { Step } from '@/types/step';

@Component({
  computed: {
    ...mapState(['puckColors', 'puckInGrip', 'currentStep', 'depositedPuck']),
  },
  methods: {
    ...mapMutations(['changeStep', 'changeGrip']),
  },
})
export default class PuckDeposit extends Vue {
  public puckColors!: Array<Color>;
  public puckInGrip!: boolean;
  public currentStep!: Step;
  public deposited: Array<Color> = [];
  public changeStep!: () => void;
  public changeGrip!: () => void;

  public testChangeStep() {
    this.changeStep();
  }

  public testChangeGrip() {
    this.changeGrip();
  }

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

<style scoped>
.v-card__title {
  padding: 0px;
}
</style>
