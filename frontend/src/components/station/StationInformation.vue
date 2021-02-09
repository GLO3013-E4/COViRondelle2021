<template>
<<<<<<< HEAD:frontend/src/components/station/StationInformations.vue
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
=======
  <v-card class="d-flex justify-center" color="#ededed" height="300">
    <h3>Station information</h3>
>>>>>>> develop:frontend/src/components/station/StationInformation.vue
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from "vue-property-decorator";
import Resistance from "../station/Resistance.vue";

@Component({
  components: { Resistance: Resistance }
})
<<<<<<< HEAD:frontend/src/components/station/StationInformations.vue
export default class StationInformations extends Vue {
  private resistanceValue = 800000; //voir l'affichage avec 3 chiffres, pas couleurs associé
  private pucksColors = ["indigo", "red", "orange"]; //Assume que je reçois un array de couleur
}
=======
export default class StationInformation extends Vue {}
>>>>>>> develop:frontend/src/components/station/StationInformation.vue
</script>

<style></style>
