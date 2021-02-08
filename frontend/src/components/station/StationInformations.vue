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
        <v-col sm="6">
          <v-card>
            <v-card-title sm="6" class="grey darken-1 d-flex justify-center">
              <h5 class="white--text">
                Station de résistance
              </h5>
            </v-card-title>
            <v-row>
              <v-card-text>
                <h5 class="text-primary">{{ this.resistanceValue }} Ω</h5>
                <v-avatar
                  size="30"
                  v-for="(puckColor, i) in pucksColors"
                  :key="i"
                  :color="puckColor"
                  >{{ i }}
                </v-avatar>
              </v-card-text>
            </v-row>
          </v-card>
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from "vue-property-decorator";

@Component({
  components: {}
})
export default class StationInformations extends Vue {
  private resistanceValue = 800; //voir l'affichage avec 3 chiffres
  private pucksColors = ["indigo", "red", "orange"]; //Assume que je reçois un array de couleur
}
</script>

<style></style>
