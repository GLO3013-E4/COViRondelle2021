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
  components: { Resistance: Resistance },
})
export default class StationInformations extends Vue {
  //TODO: par default devra être 0 (avant de recevoir info)
  private resistanceValue = 800000; //voir l'affichage avec 6 chiffres, pas couleurs associé
  //TODO: par defaut devra être liste vide (avant de recevoir info)
  private pucksColors = ["red", "blue", "orange"]; //Assume que je reçois un array de couleur
}
</script>

<style></style>
