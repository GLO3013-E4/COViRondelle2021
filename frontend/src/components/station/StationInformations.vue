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
          <Resistance
            v-bind:resistanceValue="resistanceValue"
            v-bind:pucksColors="pucksColors"
          />
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from "vue-property-decorator";
import Resistance from "../station/Resistance.vue";

@Component({
  components: { Resistance: Resistance }
})
export default class StationInformations extends Vue {
  private resistanceValue = 800; //voir l'affichage avec 3 chiffres
  private pucksColors = ["indigo", "red", "orange"]; //Assume que je reçois un array de couleur
}
</script>

<style></style>
