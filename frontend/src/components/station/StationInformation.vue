<template>
  <v-card color="#ededed" height="300">
    <v-container>
      <v-row>
        <v-col sm="12">
          <v-card class="d-flex justify-center">
            <h3>Stations informations</h3>
          </v-card>
        </v-col>
      </v-row>
      <v-row no-gutters>
        <v-col sm="5">
          <Resistance />
        </v-col>
        <v-col sm="3">
          <ControlPanel
            v-bind:controlPanelResult="controlPanelResult"
            v-bind:colorFirstPuck="colorFirstPuck"
          />
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import Resistance from '../station/Resistance.vue';
import ControlPanel from '../station/ControlPanel.vue';

//TODO: à enlever quand on recevra les vrai valeurs
import { ControlPanelResultFactory } from '../../factories/ControlPanelResult';
import { ColorFactory } from '@/factories/ColorFactory';

@Component({
  components: { Resistance: Resistance, ControlPanel: ControlPanel },
})
export default class StationInformation extends Vue {}
</script>

<style></style>
