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
              v-for="(puck, i) in depositedPuckColors"
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
import { PuckList } from '@/types/puckList';

@Component({
  computed: {
    ...mapState(['puckList', 'currentStep']),
  },
  methods: {
    ...mapMutations(['changeStep', 'changeGrip']),
  },
})
export default class PuckDeposit extends Vue {
  public puckList!: PuckList;
  public currentStep!: Step;
  public deposited: Array<Color> = [];
  public changeStep!: () => void;
  public changeGrip!: () => void;

  // TODO : Remove this
  public testChangeStep() {
    this.changeStep();
  }

  // TODO : Remove this
  public testChangeGrip() {
    this.changeGrip();
  }

  get noPuckYet(): boolean {
    return !this.puckList.hasOneGripped;
  }

  get firstPuckDeposited(): boolean {
    return this.puckList.first.isDeposited;
  }

  get secondPuckDeposited(): boolean {
    return this.puckList.get(1).isDeposited;
  }

  get thirdPuckDeposited(): boolean {
    return this.puckList.get(2).isDeposited;
  }

  get depositedPuckColors(): Array<Color> {
    return this.puckList.depositedPucks.map((puck) => puck.color);
  }
}
</script>

<style scoped>
.v-card__title {
  padding: 0px;
}
</style>
