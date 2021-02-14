<template>
  <v-card class="grey lighten-3" height="100%">
    <v-card-title sm="6" class="grey darken-1 d-flex justify-center">
      <h5 class="white--text">1er coin</h5>
    </v-card-title>
    <v-container>
      <div ref="resistanceValue" class="d-flex justify-center font-weight-bold">
        <v-badge dot :bottom="this.placementBottom" :left="this.placementLeft">
          {{ this.letterCorner }}
        </v-badge>
      </div>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import ControlPanelResult from '@/classes/ControlPanelResult';
import { Component, Vue, Prop } from 'vue-property-decorator';

@Component({
  components: {},
})
export default class ControlPanel extends Vue {
  @Prop() readonly controlPanelResult!: ControlPanelResult;

  private letterCorner = this.controlPanelResult.corner;
  private placementLeft = this.controlPanelResult.placementLeft;
  private placementBottom = this.controlPanelResult.placementBottom;
}
</script>