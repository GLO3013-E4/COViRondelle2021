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
              v-for="(puck, i) in releasedPucks"
              :key="i"
              :color="puck.color.toString()"
              class="lighten3--text font-weight-bold"
            >
              {{ puck.number }}
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
import { PuckList } from '@/types/puckList';

@Component({
  computed: {
    ...mapState(['puckList']),
  },
  methods: {
    ...mapMutations(['changeStep', 'changeGrip']),
  },
})
export default class PuckDeposit extends Vue {
  public puckList!: PuckList;
  public deposited: Array<Color> = [];
  public changeStep!: () => void;
  public changeGrip!: (hasOneGripped: boolean) => void;
  private hasOneGripped = false;

  // TODO : Remove this
  public testChangeStep() {
    this.changeStep();
  }

  // TODO : Remove this
  public testChangeGrip() {
    this.changeGrip(this.hasOneGripped);
    this.hasOneGripped = !this.hasOneGripped;
  }

  get releasedPucks() {
    return this.puckList.releasedPucks;
  }
}
</script>

<style scoped>
.v-card__title {
  padding: 0px;
}
</style>
