<template>
  <v-card class="grey lighten-3" height="100%">
    <v-card-title sm="6" class="grey darken-1 d-flex justify-center">
      <h5 class="white--text">1er coin</h5>
    </v-card-title>
    <v-container height="100%">
      <v-row align="center">
        <v-col sm="12">
          <div ref="corner" class="d-flex justify-center font-weight-bold">
            <v-badge
              dot
              :bottom="this.placementBottom"
              :left="this.placementLeft"
              :color="firstPuckColor"
            >
              {{ this.letterCorner }}
            </v-badge>
          </div>
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';
import { Corner } from '@/types/corner';
import { Color } from '@/types/color';
import ControlPanelResult from '@/classes/ControlPanelResult';

@Component({
  computed: {
    ...mapState(['puckFirstCorner', 'puckColors']),
  },
})
export default class ControlPanel extends Vue {
  private puckFirstCorner!: Corner;
  private puckColors!: Array<Color>;

  private get controlPanelResult(): ControlPanelResult {
    return new ControlPanelResult(this.puckFirstCorner);
  }

  private get letterCorner(): string {
    return this.controlPanelResult.corner;
  }

  private get placementLeft(): boolean {
    return this.controlPanelResult.placementLeft;
  }

  private get placementBottom(): boolean {
    return this.controlPanelResult.placementBottom;
  }

  private get firstPuckColor(): string {
    return this.puckColors.length > 0 ? this.puckColors[0] : '';
  }
}
</script>
